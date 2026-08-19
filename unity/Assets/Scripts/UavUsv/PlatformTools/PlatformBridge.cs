using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Scripting;

namespace UavUsv.PlatformTools
{
    /// <summary>
    /// WebGL-side receiver for authoritative ROS/Gazebo poses forwarded by Vue.
    /// ROS owns vehicle motion; Unity only converts ENU coordinates and renders it.
    /// </summary>
    [Preserve]
    [DefaultExecutionOrder(12000)]
    public sealed class PlatformBridge : MonoBehaviour
    {
        [Serializable]
        private sealed class PlatformRequest
        {
            public string runId;
            public string coordinateSystem;
            public string platform;
            public string protocolVersion;
        }

        [Serializable]
        private sealed class ScenarioRequest
        {
            public string runId;
            public string scenarioId;
            public string sceneName;
            public string coordinateSystem;
        }

        [Serializable]
        private sealed class PoseBatch
        {
            public string runId;
            public long sequence;
            public long timestamp;
            public long timestamp_ms;
            public string source;
            public string coordinateSystem;
            public string algorithmCode;
            public string phase;
            public PoseItem[] poses;
        }

        [Serializable]
        private sealed class PoseItem
        {
            public string deviceCode;
            public string type;
            public float[] position;
            public float x;
            public float y;
            public float z;
            public float[] orientation;
            public float yaw;
            public float yawDegrees;
            public bool hasOrientation;
            public bool hasYaw;
            public bool hasYawDegrees;
        }

        [Serializable]
        private sealed class MissionStateRequest
        {
            public string runId;
            public long sequence;
            public string state;
            public string phase;
            public string status;
            public string message;
        }

        [Serializable]
        private sealed class DeviceRequest
        {
            public string deviceCode;
        }

        [Serializable]
        private sealed class CameraRequest
        {
            public string mode;
            public string deviceCode;
        }

        [Serializable]
        private sealed class TrajectoryRequest
        {
            public bool visible;
        }

        [Serializable]
        private sealed class ResponseEnvelope
        {
            public string type;
            public string requestId = string.Empty;
            public long timestamp;
            public ResponsePayload payload;
        }

        [Serializable]
        private sealed class ResponsePayload
        {
            public bool success;
            public bool ready;
            public bool controlsReady;
            public bool cameraReady;
            public bool algorithmReady;
            public bool visualSensorReady;
            public string runId;
            public long sequence;
            public long timestampMs;
            public int appliedCount;
            public int missingCount;
            public int deviceCount;
            public string coordinateSystem;
            public string scenarioId;
            public string algorithmCode;
            public string phase;
            public string state;
            public string status;
            public string deviceCode;
            public string mode;
            public bool visible;
            public string buildId;
            public string source = "unity-webgl";
            public string[] capabilities;
        }

        private sealed class PoseTarget
        {
            public Transform subject;
            public Vector3 position;
            public Quaternion rotation;
            public bool initialized;
        }

        public const string BuildId = "unity-webgl-ros-pose-v4";

#if UNITY_WEBGL && !UNITY_EDITOR
        [DllImport("__Internal")]
        private static extern void VueWebGlPostMessage(string message);
#endif

        private readonly Dictionary<string, PoseTarget> targets =
            new Dictionary<string, PoseTarget>(StringComparer.OrdinalIgnoreCase);
        private string activeRunId = string.Empty;
        private string coordinateSystem = "ROS_ENU";
        private long lastSequence;
        private long lastTimestampMs;
        private float smoothing = 14f;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            GameObject existing = GameObject.Find("PlatformBridge");
            GameObject host = existing ? existing : new GameObject("PlatformBridge");
            DontDestroyOnLoad(host);
            if (!host.GetComponent<PlatformBridge>())
                host.AddComponent<PlatformBridge>();
        }

        [Preserve]
        public void InitializePlatform(string json)
        {
            PlatformRequest request = Parse<PlatformRequest>(json);
            activeRunId = request != null ? request.runId ?? string.Empty : string.Empty;
            coordinateSystem = NormalizeCoordinateSystem(
                request != null ? request.coordinateSystem : null,
                "ROS_ENU"
            );
            ResetFrameClock();
            int deviceCount = BindKnownSceneObjects();
            Post("platformInitialized", new ResponsePayload
            {
                success = deviceCount >= 6,
                ready = deviceCount >= 6,
                controlsReady = true,
                cameraReady = Camera.main,
                algorithmReady = FindObjectOfType<AlgorithmScenarioBridge>(true),
                visualSensorReady = FindObjectOfType<UnityVisualSensorBridge>(true),
                runId = activeRunId,
                deviceCount = deviceCount,
                coordinateSystem = coordinateSystem,
                status = deviceCount >= 6
                    ? "ROS pose receiver ready"
                    : "ROS pose receiver ready with missing scene objects",
                buildId = BuildId,
                capabilities = Capabilities()
            });
        }

        [Preserve]
        public void LoadScenario(string json)
        {
            ScenarioRequest request = Parse<ScenarioRequest>(json) ?? new ScenarioRequest();
            activeRunId = request.runId ?? string.Empty;
            coordinateSystem = NormalizeCoordinateSystem(request.coordinateSystem, coordinateSystem);
            ResetFrameClock();
            BindKnownSceneObjects();
            Post("scenarioLoaded", new ResponsePayload
            {
                success = true,
                runId = activeRunId,
                scenarioId = request.scenarioId ?? "default",
                algorithmCode = request.scenarioId ?? string.Empty,
                coordinateSystem = coordinateSystem,
                status = "Scenario pose receiver reset",
                buildId = BuildId
            });
        }

        [Preserve]
        public void ApplyPoseBatch(string json)
        {
            PoseBatch batch = Parse<PoseBatch>(json);
            if (batch == null || batch.poses == null)
            {
                PostPoseResult(batch, false, 0, 0, "Invalid or empty pose batch");
                return;
            }

            string incomingRunId = batch.runId ?? string.Empty;
            long timestampMs = batch.timestamp_ms > 0 ? batch.timestamp_ms : batch.timestamp;
            if (!string.Equals(incomingRunId, activeRunId, StringComparison.Ordinal))
            {
                activeRunId = incomingRunId;
                ResetFrameClock();
            }

            // A Vue component may restart its local counter while the persistent
            // WebGL iframe stays alive. Accept that only when its source timestamp
            // is newer; otherwise reject genuinely stale/out-of-order packets.
            if (batch.sequence > 0 && batch.sequence <= lastSequence)
            {
                bool sourceRestart = batch.sequence <= 3 && timestampMs > lastTimestampMs;
                if (!sourceRestart)
                {
                    PostPoseResult(batch, true, 0, 0, "Stale pose batch ignored");
                    return;
                }
                ResetFrameClock();
            }

            string frameCoordinates = NormalizeCoordinateSystem(
                batch.coordinateSystem,
                coordinateSystem
            );
            if (IsMissionSceneCoordinates(frameCoordinates) &&
                !string.IsNullOrWhiteSpace(batch.algorithmCode))
            {
                lastSequence = batch.sequence;
                lastTimestampMs = Math.Max(lastTimestampMs, timestampMs);
                PostPoseResult(batch, true, 0, 0, "Mission frame delegated to AlgorithmScenarioBridge");
                return;
            }

            BindKnownSceneObjects();
            AcquireRosAuthority();
            int applied = 0;
            int missing = 0;
            foreach (PoseItem pose in batch.poses)
            {
                if (pose == null || string.IsNullOrWhiteSpace(pose.deviceCode))
                    continue;
                if (!TryGetTarget(pose.deviceCode, out PoseTarget target))
                {
                    missing++;
                    continue;
                }

                Vector3 sourcePosition = PositionOf(pose);
                target.position = ConvertPosition(sourcePosition, frameCoordinates, pose.type);
                if (TryConvertRotation(pose, frameCoordinates, out Quaternion rotation))
                    target.rotation = rotation;
                else if (!target.initialized)
                    target.rotation = target.subject.rotation;

                if (!target.initialized ||
                    Vector3.Distance(target.subject.position, target.position) > 24f)
                {
                    target.subject.SetPositionAndRotation(target.position, target.rotation);
                    target.initialized = true;
                }
                target.subject.gameObject.SetActive(true);
                applied++;
            }

            lastSequence = batch.sequence;
            lastTimestampMs = Math.Max(lastTimestampMs, timestampMs);
            coordinateSystem = frameCoordinates;
            PostPoseResult(
                batch,
                applied > 0,
                applied,
                missing,
                applied > 0 ? "Authoritative ROS pose batch applied" : "No matching Unity scene objects"
            );
        }

        [Preserve]
        public void SetMissionState(string json)
        {
            MissionStateRequest request = Parse<MissionStateRequest>(json) ?? new MissionStateRequest();
            MultiAgentCaptureDefenseScenario scenario =
                FindObjectOfType<MultiAgentCaptureDefenseScenario>(true);
            string state = (request.state ?? request.phase ?? string.Empty).Trim().ToUpperInvariant();
            if (scenario && (state == "PAUSED" || state == "COMPLETED" ||
                             state == "FAILED" || state == "CANCELLED"))
            {
                scenario.automatic = false;
                scenario.enabled = false;
            }
            Post("missionStateChanged", new ResponsePayload
            {
                success = true,
                runId = request.runId ?? activeRunId,
                sequence = request.sequence,
                state = state,
                phase = request.phase ?? state,
                status = string.IsNullOrWhiteSpace(request.message)
                    ? "Mission state received"
                    : request.message
            });
        }

        [Preserve]
        public void SelectDevice(string json)
        {
            DeviceRequest request = Parse<DeviceRequest>(json) ?? new DeviceRequest();
            WebDeviceObserverCamera observer = EnsureObserver();
            string code = request.deviceCode ?? string.Empty;
            string profile = string.Empty;
            string error = "Unity observer camera is not ready";
            bool success = observer && observer.TrySelectDevice(
                request.deviceCode,
                out code,
                out profile,
                out error
            );
            Post("cameraChanged", new ResponsePayload
            {
                success = success,
                deviceCode = success ? code : request.deviceCode ?? string.Empty,
                mode = success ? observer.CurrentModeName : "device-follow",
                status = success ? "Camera following " + code : error
            });
        }

        [Preserve]
        public void SetCameraMode(string json)
        {
            CameraRequest request = Parse<CameraRequest>(json) ?? new CameraRequest();
            WebDeviceObserverCamera observer = EnsureObserver();
            string mode = (request.mode ?? "overview").Trim().ToLowerInvariant();
            bool success = observer;
            string deviceCode = request.deviceCode ?? string.Empty;
            string status = "Camera mode changed";
            if (!observer)
            {
                status = "Unity observer camera is not ready";
            }
            else if (mode == "overview")
            {
                observer.SetOverview();
            }
            else if (mode == "lighthouse")
            {
                observer.SetLighthouse();
            }
            else if (mode == "device-follow")
            {
                success = observer.TrySelectDevice(deviceCode, out deviceCode, out _, out status);
            }
            else if (mode == "follow-uav" || mode == "follow-usv")
            {
                success = observer.TrySelectFirst(
                    mode == "follow-uav" ? "UAV" : "USV",
                    out deviceCode,
                    out _,
                    out status
                );
                mode = "device-follow";
            }
            else
            {
                success = false;
                status = "Unknown camera mode: " + request.mode;
            }

            Post("cameraChanged", new ResponsePayload
            {
                success = success,
                deviceCode = deviceCode,
                mode = mode,
                status = status
            });
        }

        [Preserve]
        public void SetTrajectoryVisible(string json)
        {
            TrajectoryRequest request = Parse<TrajectoryRequest>(json) ?? new TrajectoryRequest();
            MultiAgentCaptureDefenseScenario scenario =
                FindObjectOfType<MultiAgentCaptureDefenseScenario>(true);
            if (scenario)
                scenario.showDebugOverlays = request.visible;
            Post("trajectoryVisibilityChanged", new ResponsePayload
            {
                success = scenario,
                visible = request.visible,
                status = request.visible ? "Trajectory overlays visible" : "Trajectory overlays hidden"
            });
        }

        private void LateUpdate()
        {
            if (targets.Count == 0)
                return;
            float alpha = 1f - Mathf.Exp(-smoothing * Time.unscaledDeltaTime);
            foreach (PoseTarget target in targets.Values)
            {
                if (target == null || !target.initialized || !target.subject)
                    continue;
                target.subject.position = Vector3.Lerp(target.subject.position, target.position, alpha);
                target.subject.rotation = Quaternion.Slerp(target.subject.rotation, target.rotation, alpha);
            }
        }

        private int BindKnownSceneObjects()
        {
            int vehicleCount = 0;
            string[] known =
            {
                "uav_01", "uav_02", "uav_03",
                "usv_01", "usv_02", "usv_03",
                "friendly_ship", "enemy_ship", "lighthouse",
                "buoy_west", "buoy_south", "buoy_east"
            };
            foreach (string name in known)
            {
                Transform subject = FindExact(name);
                if (!subject)
                    continue;
                RegisterAliases(name, subject);
                if (name.StartsWith("uav_", StringComparison.OrdinalIgnoreCase) ||
                    name.StartsWith("usv_", StringComparison.OrdinalIgnoreCase))
                    vehicleCount++;
            }
            return vehicleCount;
        }

        private void RegisterAliases(string sceneName, Transform subject)
        {
            RegisterTarget(sceneName, subject);
            RegisterTarget(sceneName.Replace('_', '-'), subject);
            string normalized = NormalizeCode(sceneName);
            if (normalized == "enemyship")
            {
                RegisterTarget("target", subject);
                RegisterTarget("target_vessel", subject);
                RegisterTarget("capture-target", subject);
                RegisterTarget("threat-target", subject);
            }
            else if (normalized == "friendlyship")
            {
                RegisterTarget("escort-target", subject);
                RegisterTarget("escort_target", subject);
            }
        }

        private void RegisterTarget(string code, Transform subject)
        {
            string key = NormalizeCode(code);
            if (string.IsNullOrEmpty(key) || targets.ContainsKey(key))
                return;
            targets[key] = new PoseTarget
            {
                subject = subject,
                position = subject.position,
                rotation = subject.rotation
            };
        }

        private bool TryGetTarget(string code, out PoseTarget target)
        {
            string key = NormalizeCode(code);
            if (targets.TryGetValue(key, out target) && target.subject)
                return true;
            Transform subject = FindExact(code) ?? FindExact(code.Replace('-', '_'));
            if (!subject)
                return false;
            RegisterTarget(code, subject);
            target = targets[key];
            return true;
        }

        private static Transform FindExact(string objectName)
        {
            if (string.IsNullOrWhiteSpace(objectName))
                return null;
            foreach (Transform item in FindObjectsOfType<Transform>(true))
                if (item && string.Equals(item.name, objectName, StringComparison.OrdinalIgnoreCase))
                    return item;
            return null;
        }

        private void AcquireRosAuthority()
        {
            MultiAgentCaptureDefenseScenario scenario =
                FindObjectOfType<MultiAgentCaptureDefenseScenario>(true);
            if (!scenario)
                return;
            scenario.automatic = false;
            scenario.enabled = false;
        }

        private static Vector3 PositionOf(PoseItem pose)
        {
            if (pose.position != null && pose.position.Length >= 3)
                return new Vector3(pose.position[0], pose.position[1], pose.position[2]);
            return new Vector3(pose.x, pose.y, pose.z);
        }

        private static Vector3 ConvertPosition(Vector3 position, string coordinates, string type)
        {
            if (IsRosCoordinates(coordinates))
                return Coordinates.ToPresentation(position.x, position.y, position.z);
            if (IsMissionSceneCoordinates(coordinates))
            {
                UnityScenarioCompatibilityInstaller installer =
                    FindObjectOfType<UnityScenarioCompatibilityInstaller>(true);
                Vector3 origin = installer && installer.Ready
                    ? installer.MissionOrigin
                    : Vector3.zero;
                bool isUav = string.Equals(type, "UAV", StringComparison.OrdinalIgnoreCase);
                float height = isUav ? Mathf.Max(2f, position.z) : position.z;
                return origin + new Vector3(position.x, height, position.y);
            }
            return position;
        }

        private static bool TryConvertRotation(
            PoseItem pose,
            string coordinates,
            out Quaternion rotation)
        {
            bool hasQuaternion = pose.orientation != null && pose.orientation.Length >= 4 &&
                                 (pose.hasOrientation || QuaternionMagnitude(pose.orientation) > .0001f);
            if (hasQuaternion)
            {
                if (IsRosCoordinates(coordinates))
                {
                    rotation = new Quaternion(
                        -pose.orientation[0],
                        -pose.orientation[2],
                        -pose.orientation[1],
                        pose.orientation[3]
                    ).normalized;
                }
                else
                {
                    rotation = new Quaternion(
                        pose.orientation[0],
                        pose.orientation[1],
                        pose.orientation[2],
                        pose.orientation[3]
                    ).normalized;
                }
                return true;
            }

            if (pose.hasYawDegrees)
            {
                rotation = Quaternion.Euler(0f, IsRosCoordinates(coordinates)
                    ? -pose.yawDegrees
                    : pose.yawDegrees, 0f);
                return true;
            }
            if (pose.hasYaw)
            {
                float degrees = pose.yaw * Mathf.Rad2Deg;
                rotation = Quaternion.Euler(0f, IsRosCoordinates(coordinates) ? -degrees : degrees, 0f);
                return true;
            }

            rotation = Quaternion.identity;
            return false;
        }

        private static float QuaternionMagnitude(float[] q)
        {
            if (q == null || q.Length < 4)
                return 0f;
            return q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        }

        private static bool IsRosCoordinates(string value)
        {
            string normalized = (value ?? string.Empty).Trim().ToUpperInvariant();
            return normalized.Contains("ROS") || normalized.Contains("ENU") ||
                   normalized.Contains("GAZEBO");
        }

        private static bool IsMissionSceneCoordinates(string value)
        {
            return (value ?? string.Empty).Trim().ToUpperInvariant().Contains("MISSION_SCENE");
        }

        private static string NormalizeCoordinateSystem(string value, string fallback)
        {
            return string.IsNullOrWhiteSpace(value)
                ? (string.IsNullOrWhiteSpace(fallback) ? "ROS_ENU" : fallback)
                : value.Trim().ToUpperInvariant();
        }

        private static string NormalizeCode(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return string.Empty;
            char[] buffer = new char[value.Length];
            int cursor = 0;
            foreach (char item in value.Trim().ToLowerInvariant())
                if (char.IsLetterOrDigit(item))
                    buffer[cursor++] = item;
            return new string(buffer, 0, cursor);
        }

        private WebDeviceObserverCamera EnsureObserver()
        {
            Camera camera = Camera.main;
            if (!camera)
                return null;
            WebDeviceObserverCamera observer = camera.GetComponent<WebDeviceObserverCamera>();
            if (!observer)
                observer = camera.gameObject.AddComponent<WebDeviceObserverCamera>();
            observer.Initialize(camera, camera.GetComponent<ChaseCamera>());
            return observer;
        }

        private void ResetFrameClock()
        {
            lastSequence = 0;
            lastTimestampMs = 0;
        }

        private void PostPoseResult(
            PoseBatch batch,
            bool success,
            int applied,
            int missing,
            string status)
        {
            Post("poseFrameApplied", new ResponsePayload
            {
                success = success,
                runId = batch != null ? batch.runId ?? activeRunId : activeRunId,
                sequence = batch != null ? batch.sequence : 0,
                timestampMs = batch != null
                    ? (batch.timestamp_ms > 0 ? batch.timestamp_ms : batch.timestamp)
                    : 0,
                appliedCount = applied,
                missingCount = missing,
                coordinateSystem = batch != null
                    ? NormalizeCoordinateSystem(batch.coordinateSystem, coordinateSystem)
                    : coordinateSystem,
                algorithmCode = batch != null ? batch.algorithmCode ?? string.Empty : string.Empty,
                phase = batch != null ? batch.phase ?? string.Empty : string.Empty,
                status = status
            });
        }

        private static T Parse<T>(string json) where T : class
        {
            if (string.IsNullOrWhiteSpace(json))
                return null;
            try
            {
                return JsonUtility.FromJson<T>(json);
            }
            catch (Exception exception)
            {
                Debug.LogWarning("[PlatformBridge] Invalid JSON: " + exception.Message);
                return null;
            }
        }

        private static string[] Capabilities()
        {
            return new[]
            {
                "InitializePlatform", "LoadScenario", "ApplyPoseBatch",
                "SetMissionState", "SelectDevice", "SetCameraMode",
                "SetTrajectoryVisible", "ros-pose-receiver", "ros-enu",
                "camera-control", "vehicle-control", "trajectory-telemetry",
                "algorithm-scenario", "visual-sensor"
            };
        }

        private static void Post(string type, ResponsePayload payload)
        {
            var envelope = new ResponseEnvelope
            {
                type = type,
                timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                payload = payload
            };
            string json = JsonUtility.ToJson(envelope);
#if UNITY_WEBGL && !UNITY_EDITOR
            VueWebGlPostMessage(json);
#else
            Debug.Log("[PlatformBridge] " + json);
#endif
        }
    }
}
