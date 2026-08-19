<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useRoute, useRouter } from 'vue-router'

import MissionEventDrawer from '@/components/mission/MissionEventDrawer.vue'
import MissionExecutionOverlay from '@/components/mission/MissionExecutionOverlay.vue'
import type { VehicleQuickCommand } from '@/components/control/VehicleQuickControl.vue'
import { executeMissionAction, fetchMission } from '@/api/mission'
import { issueRuntimeCommand } from '@/api/runtimeControl'
import type { RuntimeCommandStatus, RuntimeCommandType } from '@/api/runtimeControl'
import { useMissionTrajectorySessionStore } from '@/stores/missionTrajectorySession'
import { useMonitoringStore } from '@/stores/monitoring'
import { useRealtimeStore } from '@/stores/realtime'
import { trajectoryToAlgorithmFrame, useTrajectoryStore } from '@/stores/trajectory'
import { useUnityBridgeStore } from '@/stores/unityBridge'
import { useUnityViewportStore } from '@/stores/unityViewport'
import { useVisualSensorStore } from '@/stores/visualSensor'
import type { AlgorithmRuntimeFrame, MissionDetail } from '@/types/mission'
import type { RuntimeNode } from '@/types/monitoring'
import type { VisualSensorRuntimeContext } from '@/types/visualSensor'

const route = useRoute()
const router = useRouter()
const monitoringStore = useMonitoringStore()
const realtimeStore = useRealtimeStore()
const trajectoryStore = useTrajectoryStore()
const unityBridgeStore = useUnityBridgeStore()
const sessionStore = useMissionTrajectorySessionStore()
const unityViewportStore = useUnityViewportStore()
const visualSensorStore = useVisualSensorStore()

const detail = ref<MissionDetail | null>(null)
const selectedDeviceCode = ref('uav-01')
const commandFeedback = ref<Record<string, RuntimeCommandStatus | undefined>>({})
const operationalStates = ref<Record<string, string | undefined>>({})
const busy = ref(false)
const eventVisible = ref(false)
const mode = ref<'2d' | '3d' | 'vision'>('2d')
const visualDisplayMode = ref<'grid' | 'focus'>('grid')
const missionId = computed(() => Number(route.params.missionId))
const runId = computed(() => Number(route.params.runId))
const unityChannel = computed(() => unityBridgeStore.channels.MISSION_CENTER)
const trajectoryFrame = computed(() => trajectoryStore.channels.MISSION_CENTER.frame)
const algorithmFrame = computed<AlgorithmRuntimeFrame | null>(() => trajectoryToAlgorithmFrame(
  trajectoryFrame.value,
  detail.value?.currentRun?.id ?? null,
  detail.value?.mission.algorithmCode,
))
const missionVisualStats = computed(() => visualSensorStore.streamStatsFor('MISSION_CENTER'))
const missionVisualConnected = computed(() =>
  visualSensorStore.unityBridgeReadyFor('MISSION_CENTER')
  && missionVisualStats.value?.active === true
  && visualSensorStore.runtimeContextFor('MISSION_CENTER').runId === runId.value,
)
const unityRunSynchronized = computed(() =>
  unityChannel.value.connected
  && !!trajectoryFrame.value
  && Date.now() - trajectoryFrame.value.receivedAt < 3_000,
)
let loadedScenarioKey = ''

function missionVisualContext(): VisualSensorRuntimeContext {
  return {
    runtimeScope: 'MISSION_CENTER',
    runtimeInstanceId: unityViewportStore.missionInstanceId,
    missionId: detail.value?.mission.id ?? (Number.isFinite(missionId.value) ? missionId.value : null),
    runId: detail.value?.currentRun?.id ?? (Number.isFinite(runId.value) ? runId.value : null),
  }
}

function cameraIdForDevice(deviceCode: string) {
  return deviceCode.trim().toLowerCase().replace(/-/g, '_')
}

function sendMissionVisualSubscription(
  enabled: boolean,
  displayMode: 'grid' | 'focus' | 'off' = visualDisplayMode.value,
) {
  const context = missionVisualContext()
  if (context.missionId === null || context.runId === null) return
  visualSensorStore.bindRuntime(context)
  const focusedCameraId = cameraIdForDevice(selectedDeviceCode.value || 'uav-01')
  void visualSensorStore.selectFor('MISSION_CENTER', focusedCameraId)
  unityBridgeStore.sendFor('MISSION_CENTER', 'visualSensorSubscribe', {
    enabled,
    missionId: context.missionId,
    runId: context.runId,
    runtimeInstanceId: context.runtimeInstanceId,
    focusedCameraId,
    displayMode,
    quality: '720p',
    targetFps: 30,
    gpuDirect: true,
    jpegFallback: false,
    thumbnailFps: 0.2,
    focusedFps: 1,
  })
}

function selectDevice(deviceCode: string) {
  selectedDeviceCode.value = deviceCode
  if (mode.value !== 'vision') return
  visualDisplayMode.value = 'focus'
  sendMissionVisualSubscription(true, 'focus')
}

function showVisualGrid() {
  visualDisplayMode.value = 'grid'
  sendMissionVisualSubscription(true, 'grid')
}

function requestedViewMode(): '2d' | '3d' | 'vision' {
  return route.query.view === 'vision'
    ? 'vision'
    : route.query.view === '3d'
      ? '3d'
      : '2d'
}

function ensureMissionScenarioLoaded() {
  if (!unityChannel.value.controlsReady) {
    loadedScenarioKey = ''
    return
  }
  const mission = detail.value?.mission
  const currentRun = detail.value?.currentRun
  if (!mission || !currentRun || !['GB_SFLA_CS', 'ESCORT_GUARD'].includes(mission.algorithmCode)) return
  const key = `${mission.id}:${currentRun.id}:${mission.algorithmCode}:${unityViewportStore.missionInstanceId}`
  if (loadedScenarioKey === key) return
  unityBridgeStore.sendFor('MISSION_CENTER', 'loadScenario', {
    algorithmCode: mission.algorithmCode,
    missionId: mission.id,
    runId: currentRun.id,
  })
  loadedScenarioKey = key
}

const runtimeNodes = computed<RuntimeNode[]>(() => {
  const frame = trajectoryFrame.value
  if (!frame) return monitoringStore.nodes.filter(node => node.type === 'UAV' || node.type === 'USV')
  return frame.agents
    .filter(agent => agent.type === 'UAV' || agent.type === 'USV')
    .map((agent, index) => {
      const existing = monitoringStore.nodes.find(node => node.code.toLowerCase() === agent.code.toLowerCase())
      return {
        id: existing?.id ?? -(index + 1),
        code: agent.code,
        name: existing?.name ?? `协同${agent.type === 'UAV' ? '无人机' : '无人艇'} ${agent.code.replace(/[^0-9]/g, '')}`,
        type: agent.type as 'UAV' | 'USV',
        status: 'ONLINE',
        host: null,
        port: null,
        endpoint: 'ros://fleet/state',
        rosNamespace: null,
        lastHeartbeatAt: new Date(frame.receivedAt).toISOString(),
        heartbeatAgeSeconds: Math.max(0, Math.round((Date.now() - frame.receivedAt) / 1000)),
        source: 'ROS_GAZEBO',
        instanceId: unityViewportStore.missionInstanceId,
        positionX: agent.x,
        positionY: agent.y,
        positionZ: agent.z,
        orientationX: null,
        orientationY: null,
        orientationZ: null,
        orientationW: null,
        detail: agent.state,
      }
    })
})

watch(trajectoryFrame, frame => {
  if (!frame) return
  const next = { ...operationalStates.value }
  for (const agent of frame.agents) {
    if (agent.type === 'UAV' || agent.type === 'USV') next[agent.code.toLowerCase()] = agent.state
  }
  operationalStates.value = next
  if (!runtimeNodes.value.some(node => node.code.toLowerCase() === selectedDeviceCode.value.toLowerCase())) {
    selectDevice(runtimeNodes.value[0]?.code ?? '')
  }
}, { immediate: true })

function runtimeCommandAction(commandType: RuntimeCommandType) {
  const commands: Partial<Record<RuntimeCommandType, string>> = {
    UAV_TAKEOFF: 'uavTakeoff', UAV_HOVER: 'uavHover', UAV_RESUME: 'uavResume', UAV_RETURN: 'uavReturn', UAV_LAND: 'uavLand', UAV_EMERGENCY_LAND: 'uavEmergencyLand',
    USV_DEPART: 'usvDepart', USV_HOLD: 'usvHold', USV_RESUME: 'usvResume', USV_RETURN: 'usvReturn', USV_STOP: 'usvStop', USV_EMERGENCY_STOP: 'usvEmergencyStop',
  }
  return commands[commandType] ?? commandType.toLowerCase()
}

async function loadDetail() {
  if (!Number.isFinite(missionId.value) || !Number.isFinite(runId.value)) throw new Error('任务运行地址无效')
  const loaded = await fetchMission(missionId.value)
  const requestedRun = loaded.runs.find(run => run.id === runId.value) ?? (loaded.currentRun?.id === runId.value ? loaded.currentRun : null)
  if (!requestedRun) throw new Error('未找到该任务运行批次')
  loaded.currentRun = requestedRun
  if (detail.value?.currentRun?.id !== requestedRun.id) {
    loadedScenarioKey = ''
    trajectoryStore.clearFor('MISSION_CENTER')
  }
  detail.value = loaded
  sessionStore.bind(loaded.mission.id, requestedRun.id)
  unityViewportStore.prepareMission(loaded.mission.id, requestedRun.id, requestedRun.runtimeInstanceId)
  visualSensorStore.bindRuntime(missionVisualContext())
}

async function refreshUntilStatus(expected: string) {
  for (let attempt = 0; attempt < 12; attempt += 1) {
    await loadDetail()
    if (detail.value?.mission.status === expected) return
    await new Promise(resolve => window.setTimeout(resolve, 400))
  }
}

watch([
  () => unityChannel.value.controlsReady,
  () => detail.value?.mission.algorithmCode,
  () => detail.value?.currentRun?.id,
  () => unityViewportStore.missionInstanceId,
], ensureMissionScenarioLoaded, { immediate: true })

async function runMissionAction(action: 'pause' | 'resume' | 'complete' | 'cancel') {
  if (!detail.value) return
  if (action === 'cancel') {
    try {
      await ElMessageBox.confirm('确认终止当前任务运行？该操作只影响任务中心，不影响系统总览。', '终止任务', { type: 'warning', confirmButtonText: '确认终止', cancelButtonText: '取消' })
    } catch { return }
  }
  busy.value = true
  try {
    const result = await executeMissionAction(detail.value.mission.id, action, 'MISSION_CONTROL', unityViewportStore.missionInstanceId)
    if (result.command) {
      if (result.command.status === 'FAILED' || result.command.status === 'TIMEOUT') throw new Error(result.command.detail || '任务指令创建失败')
      if (result.command.status !== 'ACKNOWLEDGED') {
        await realtimeStore.waitForCommandAck(result.command.commandKey)
      }
    }
    if (action === 'pause') {
      sessionStore.pause()
    }
    if (action === 'resume') {
      sessionStore.resume(trajectoryFrame.value?.sequence ?? 0)
    }
    if (action === 'complete' || action === 'cancel') {
      sessionStore.stop()
    }
    const expectedStatus = action === 'pause'
      ? 'PAUSED'
      : action === 'resume'
        ? 'RUNNING'
        : action === 'complete'
          ? 'COMPLETED'
          : 'CANCELLED'
    // The backend applies mission state from the ROS acknowledgement event.
    // Reflect that acknowledged state immediately, then reconcile with the
    // persisted mission record in the background.
    detail.value = {
      ...detail.value,
      mission: { ...detail.value.mission, status: expectedStatus as typeof detail.value.mission.status },
      currentRun: detail.value.currentRun
        ? { ...detail.value.currentRun, status: expectedStatus as typeof detail.value.currentRun.status }
        : null,
    }
    void refreshUntilStatus(expectedStatus).catch(() => undefined)
    ElMessage.success(action === 'pause' ? 'ROS 任务已暂停' : action === 'resume' ? 'ROS 任务已继续' : action === 'complete' ? 'ROS 任务已完成' : 'ROS 任务已终止')
  } catch (error) {
    ElMessage.error(error instanceof Error ? error.message : '任务指令执行失败')
  } finally { busy.value = false }
}

async function resetScene() {
  if (!detail.value || !unityChannel.value.controlsReady) {
    ElMessage.error('Unity 与 ROS 场景尚未连接')
    return
  }
  try {
    await ElMessageBox.confirm(
      '将全部无人机、无人艇、友方船和敌方船恢复到本地 Unity 初始位置，并保持任务暂停。是否继续？',
      '恢复初始位置',
      { type: 'warning', confirmButtonText: '确认恢复', cancelButtonText: '取消' },
    )
  } catch { return }
  busy.value = true
  try {
    unityBridgeStore.sendFor('MISSION_CENTER', 'resetScene', {
      missionId: detail.value.mission.id,
      runId: detail.value.currentRun?.id ?? null,
    })
    unityBridgeStore.sendFor('MISSION_CENTER', 'cameraControl', { action: 'fitAll' })
    ElMessage.success('已请求 ROS/Gazebo 恢复本地 Unity 初始位置')
  } finally {
    window.setTimeout(() => { busy.value = false }, 1200)
  }
}

async function placeThreat(x: number, y: number) {
  if (detail.value?.mission.algorithmCode !== 'ESCORT_GUARD') return
  ElMessage.info(`威胁点 ${x.toFixed(1)}, ${y.toFixed(1)} 未修改：护航目标只接受 ROS 感知/规划数据`)
}

async function sendVehicleCommand(command: VehicleQuickCommand) {
  if (!detail.value?.currentRun || !command.deviceCodes.length) return
  busy.value = true
  let acknowledged = 0
  try {
    for (const code of command.deviceCodes) {
      const key = code.toLowerCase()
      commandFeedback.value = { ...commandFeedback.value, [key]: 'PENDING' }
      try {
        const result = await issueRuntimeCommand({
          commandType: command.commandType,
          runId: detail.value.currentRun.id,
          deviceCode: key,
          payload: JSON.stringify({ source: 'MISSION_CONTROL', action: runtimeCommandAction(command.commandType) }),
          detail: `${command.label} / ${key}`,
          runtimeScope: 'MISSION_CENTER',
          runtimeInstanceId: unityViewportStore.missionInstanceId,
        })
        if (result.status === 'FAILED' || result.status === 'TIMEOUT') throw new Error(result.detail)
        let success = result.status === 'ACKNOWLEDGED'
        if (!success) {
          const ack = await realtimeStore.waitForCommandAck(result.commandKey)
          success = ack.status === 1 || ack.status === 3
        }
        commandFeedback.value = { ...commandFeedback.value, [key]: success ? 'ACKNOWLEDGED' : 'FAILED' }
        if (success) acknowledged += 1
      } catch {
        commandFeedback.value = { ...commandFeedback.value, [key]: 'FAILED' }
      }
    }
    if (acknowledged === command.deviceCodes.length) ElMessage.success(`${command.label}：${acknowledged}/${command.deviceCodes.length} 台已确认`)
    else ElMessage.error(`${command.label}：成功 ${acknowledged}，失败 ${command.deviceCodes.length - acknowledged}`)
  } finally { busy.value = false }
}

function changeMode(next: '2d' | '3d' | 'vision') {
  const leavingVision = mode.value === 'vision' && next !== 'vision'
  mode.value = next
  if (leavingVision) sendMissionVisualSubscription(false, 'off')
  if (next === 'vision') {
    visualDisplayMode.value = 'grid'
    unityViewportStore.show('mission-execution')
    sendMissionVisualSubscription(true, 'grid')
    return
  }
  if (next === '3d') {
    unityViewportStore.show('mission-execution')
    return
  }
  unityViewportStore.park()
}

async function closeExecution() {
  unityViewportStore.park()
  await router.push({ name: 'missions' })
}

onMounted(async () => {
  unityViewportStore.park()
  monitoringStore.connectEvents()
  await monitoringStore.refresh({}, true)
  try {
    await loadDetail()
    changeMode(requestedViewMode())
  } catch (error) {
    ElMessage.error(error instanceof Error ? error.message : '任务运行加载失败')
    await router.replace({ name: 'missions' })
  }
})
onBeforeUnmount(() => {
  if (mode.value === 'vision') sendMissionVisualSubscription(false, 'off')
  visualSensorStore.disposeFrames('MISSION_CENTER')
  unityViewportStore.park()
})
</script>

<template>
  <MissionExecutionOverlay
    v-if="detail"
    :detail="detail"
    :nodes="runtimeNodes"
    :trajectory-frame="trajectoryFrame"
    :algorithm-frame="algorithmFrame"
    :session-state="sessionStore.state"
    :session-revision="sessionStore.revision"
    :selected-device-code="selectedDeviceCode"
    :feedback="commandFeedback"
    :operational-states="operationalStates"
    :mode="mode"
    :visual-display-mode="visualDisplayMode"
    :visual-connected="missionVisualConnected"
    :unity-run-synchronized="unityRunSynchronized"
    :busy="busy"
    @close="closeExecution"
    @select="selectDevice"
    @vehicle-command="sendVehicleCommand"
    @mission-action="runMissionAction"
    @reset-scene="resetScene"
    @events="eventVisible = true"
    @mode-change="changeMode"
    @visual-grid="showVisualGrid"
    @place-threat="placeThreat"
  />
  <MissionEventDrawer v-model="eventVisible" :mission-id="detail?.mission.id ?? null" :run-id="detail?.currentRun?.id" />
</template>
