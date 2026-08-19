package com.uavusv.platform.module.mission.service;

import com.uavusv.platform.module.mission.entity.MissionEvent;
import com.uavusv.platform.module.mission.entity.MissionEventLevel;
import com.uavusv.platform.module.mission.entity.MissionEventType;
import com.uavusv.platform.module.mission.entity.MissionRun;
import com.uavusv.platform.module.mission.entity.MissionRunStatus;
import com.uavusv.platform.module.mission.entity.MissionStage;
import com.uavusv.platform.module.mission.entity.MissionStatus;
import com.uavusv.platform.module.mission.entity.MissionTask;
import com.uavusv.platform.module.mission.event.RosMissionStateObservedEvent;
import com.uavusv.platform.module.mission.repository.MissionEventRepository;
import com.uavusv.platform.module.mission.repository.MissionRunRepository;
import com.uavusv.platform.module.mission.repository.MissionTaskRepository;
import org.springframework.context.event.EventListener;
import org.springframework.stereotype.Component;
import org.springframework.transaction.annotation.Transactional;

import java.util.EnumSet;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Component
public class RosMissionStateSynchronizer {

    private static final EnumSet<MissionRunStatus> OPEN_RUN_STATUSES = EnumSet.of(
            MissionRunStatus.PENDING,
            MissionRunStatus.RUNNING,
            MissionRunStatus.PAUSED
    );

    private final MissionRunRepository missionRunRepository;
    private final MissionTaskRepository missionTaskRepository;
    private final MissionEventRepository missionEventRepository;
    private final Map<String, String> lastSignatures = new ConcurrentHashMap<>();

    public RosMissionStateSynchronizer(
            MissionRunRepository missionRunRepository,
            MissionTaskRepository missionTaskRepository,
            MissionEventRepository missionEventRepository
    ) {
        this.missionRunRepository = missionRunRepository;
        this.missionTaskRepository = missionTaskRepository;
        this.missionEventRepository = missionEventRepository;
    }

    @EventListener
    @Transactional
    public void observe(RosMissionStateObservedEvent event) {
        String algorithmCode = normalize(event.algorithmCode());
        String status = normalize(event.status());
        if (algorithmCode.isBlank() || status.isBlank() || "IDLE".equals(status)) return;

        // A platform-originated command is committed by MissionCommandCoordinator
        // after its correlated ACK. Avoid racing that command with the same ROS
        // observation. Frames without a command correlation are ROS-initiated and
        // are applied here directly.
        if (event.confirmedCommandKey() != null && !event.confirmedCommandKey().isBlank()) return;

        String signature = String.join("|", status, normalize(event.phase()),
                event.targetId() == null ? "" : event.targetId(),
                event.reason() == null ? "" : event.reason());
        if (signature.equals(lastSignatures.put(algorithmCode, signature))) return;

        MissionRun run = missionRunRepository
                .findFirstByAlgorithmCodeIgnoreCaseAndStatusInOrderByStartedAtDesc(
                        algorithmCode, OPEN_RUN_STATUSES)
                .orElse(null);
        MissionTask mission = run == null
                ? null
                : missionTaskRepository.findByIdAndDeletedFalse(run.getMissionId()).orElse(null);

        if (run == null && "RUNNING".equals(status)) {
            mission = missionTaskRepository
                    .findFirstByAlgorithmCodeIgnoreCaseAndDeletedFalseAndStatusOrderByPriorityAscIdAsc(
                            algorithmCode, MissionStatus.READY)
                    .orElse(null);
            if (mission == null) return;
            MissionStage stage = stageOf(event.phase(), mission.getStage());
            run = missionRunRepository.save(new MissionRun(
                    mission.getId(),
                    null,
                    missionRunRepository.findMaxRunNo(mission.getId()) + 1,
                    stage,
                    "ROS",
                    "ros-external",
                    mission.getAlgorithmCode(),
                    mission.getAlgorithmVersion()
            ));
        }
        if (run == null || mission == null) return;

        MissionStage stage = stageOf(event.phase(), mission.getStage());
        boolean changed = apply(status, stage, event.reason(), mission, run);
        if (!changed) return;

        missionEventRepository.save(new MissionEvent(
                mission.getId(),
                run.getId(),
                MissionEventType.ROS,
                mission.getStage(),
                "FAILED".equals(status) ? MissionEventLevel.ERROR : MissionEventLevel.INFO,
                "ROS 主动任务状态：" + status,
                String.join(" / ",
                        event.phase() == null ? "" : event.phase(),
                        event.targetId() == null ? "" : event.targetId(),
                        event.reason() == null ? "" : event.reason()),
                "ROS"
        ));
    }

    private boolean apply(
            String status,
            MissionStage stage,
            String reason,
            MissionTask mission,
            MissionRun run
    ) {
        return switch (status) {
            case "RUNNING" -> {
                if (run.getStatus() == MissionRunStatus.PENDING) run.activate(stage);
                else if (run.getStatus() == MissionRunStatus.PAUSED) run.resume(stage);
                else if (run.getStatus() != MissionRunStatus.RUNNING) yield false;
                mission.updateStatus(MissionStatus.RUNNING, stage);
                yield true;
            }
            case "PAUSED" -> {
                if (run.getStatus() != MissionRunStatus.RUNNING) yield false;
                run.pause(stage);
                mission.updateStatus(MissionStatus.PAUSED, stage);
                yield true;
            }
            case "COMPLETED" -> {
                run.complete(MissionStage.EVALUATION);
                mission.updateStatus(MissionStatus.COMPLETED, MissionStage.EVALUATION);
                yield true;
            }
            case "FAILED" -> {
                run.fail(stage, reason == null || reason.isBlank() ? "ROS 任务失败" : reason);
                mission.updateStatus(MissionStatus.FAILED, stage);
                yield true;
            }
            case "CANCELLED" -> {
                run.cancel(MissionStage.EVALUATION);
                mission.updateStatus(MissionStatus.CANCELLED, MissionStage.EVALUATION);
                yield true;
            }
            default -> false;
        };
    }

    private MissionStage stageOf(String rawPhase, MissionStage fallback) {
        return switch (normalize(rawPhase)) {
            case "SEARCH" -> MissionStage.TARGET_DETECTED;
            case "TRACKING", "APPROACHING", "ESCORTING", "NORMAL_ESCORT" -> MissionStage.TRACKING;
            case "FORMING", "TAKING_OFF", "WAITING_FOR_VEHICLES", "WAITING_FOR_TARGET" ->
                    MissionStage.ASSIGNMENT;
            case "ENCIRCLING", "HOLDING", "GUARDING" -> MissionStage.ENCIRCLEMENT;
            case "SUCCESS", "COMPLETED" -> MissionStage.CAPTURED;
            case "FAILED", "CANCELLED" -> MissionStage.EVALUATION;
            default -> fallback == null ? MissionStage.PREPARE : fallback;
        };
    }

    private String normalize(String value) {
        return value == null ? "" : value.trim().toUpperCase();
    }
}
