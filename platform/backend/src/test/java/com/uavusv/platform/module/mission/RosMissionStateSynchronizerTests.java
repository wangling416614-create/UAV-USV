package com.uavusv.platform.module.mission;

import com.uavusv.platform.module.mission.entity.MissionExecutionMode;
import com.uavusv.platform.module.mission.entity.MissionRun;
import com.uavusv.platform.module.mission.entity.MissionRunStatus;
import com.uavusv.platform.module.mission.entity.MissionStage;
import com.uavusv.platform.module.mission.entity.MissionStatus;
import com.uavusv.platform.module.mission.entity.MissionTask;
import com.uavusv.platform.module.mission.entity.MissionType;
import com.uavusv.platform.module.mission.event.RosMissionStateObservedEvent;
import com.uavusv.platform.module.mission.repository.MissionEventRepository;
import com.uavusv.platform.module.mission.repository.MissionRunRepository;
import com.uavusv.platform.module.mission.repository.MissionTaskRepository;
import com.uavusv.platform.module.mission.service.RosMissionStateSynchronizer;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyCollection;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

class RosMissionStateSynchronizerTests {

    @Test
    void shouldCreateExternalRunWhenRosStartsReadyMission() {
        Fixture fixture = fixture();
        when(fixture.runRepository
                .findFirstByAlgorithmCodeIgnoreCaseAndStatusInOrderByStartedAtDesc(
                        any(), anyCollection()))
                .thenReturn(Optional.empty());
        when(fixture.taskRepository
                .findFirstByAlgorithmCodeIgnoreCaseAndDeletedFalseAndStatusOrderByPriorityAscIdAsc(
                        "ESCORT_GUARD", MissionStatus.READY))
                .thenReturn(Optional.of(fixture.mission));
        when(fixture.runRepository.findMaxRunNo(null)).thenReturn(2);
        when(fixture.runRepository.save(any(MissionRun.class)))
                .thenAnswer(invocation -> invocation.getArgument(0));

        fixture.synchronizer.observe(event(
                "ESCORT_GUARD", "RUNNING", "FORMING", null));

        ArgumentCaptor<MissionRun> runCaptor = ArgumentCaptor.forClass(MissionRun.class);
        verify(fixture.runRepository).save(runCaptor.capture());
        MissionRun run = runCaptor.getValue();
        assertEquals(3, run.getRunNo());
        assertEquals("ROS", run.getRequestedBy());
        assertEquals("ros-external", run.getRuntimeInstanceId());
        assertEquals("ESCORT_GUARD", run.getAlgorithmCode());
        assertEquals(MissionRunStatus.RUNNING, run.getStatus());
        assertEquals(MissionStage.ASSIGNMENT, run.getStage());
        assertEquals(MissionStatus.RUNNING, fixture.mission.getStatus());
        verify(fixture.eventRepository).save(any());
    }

    @Test
    void shouldCompleteExistingRunFromRosState() {
        Fixture fixture = fixture();
        MissionRun run = new MissionRun(
                null, null, 1, MissionStage.TRACKING, "ROS",
                "ros-external", "ESCORT_GUARD", "1.0.0");
        run.activate(MissionStage.TRACKING);
        when(fixture.runRepository
                .findFirstByAlgorithmCodeIgnoreCaseAndStatusInOrderByStartedAtDesc(
                        any(), anyCollection()))
                .thenReturn(Optional.of(run));
        when(fixture.taskRepository.findByIdAndDeletedFalse(null))
                .thenReturn(Optional.of(fixture.mission));

        fixture.synchronizer.observe(event(
                "ESCORT_GUARD", "COMPLETED", "COMPLETED", null));

        assertEquals(MissionRunStatus.COMPLETED, run.getStatus());
        assertEquals(MissionStage.EVALUATION, run.getStage());
        assertEquals(MissionStatus.COMPLETED, fixture.mission.getStatus());
        verify(fixture.eventRepository).save(any());
    }

    @Test
    void shouldLeavePlatformConfirmedStateToCommandCoordinator() {
        Fixture fixture = fixture();

        fixture.synchronizer.observe(event(
                "ESCORT_GUARD", "RUNNING", "FORMING", "command-42"));

        verify(fixture.runRepository, never())
                .findFirstByAlgorithmCodeIgnoreCaseAndStatusInOrderByStartedAtDesc(
                        any(), anyCollection());
        verify(fixture.eventRepository, never()).save(any());
        assertEquals(MissionStatus.READY, fixture.mission.getStatus());
    }

    private RosMissionStateObservedEvent event(
            String algorithmCode,
            String status,
            String phase,
            String confirmedCommandKey
    ) {
        return new RosMissionStateObservedEvent(
                algorithmCode,
                status,
                phase,
                "friendly_ship",
                "ROS state test",
                System.currentTimeMillis(),
                confirmedCommandKey,
                confirmedCommandKey == null ? null : "START_MISSION"
        );
    }

    private Fixture fixture() {
        MissionRunRepository runRepository = mock(MissionRunRepository.class);
        MissionTaskRepository taskRepository = mock(MissionTaskRepository.class);
        MissionEventRepository eventRepository = mock(MissionEventRepository.class);
        MissionTask mission = new MissionTask("MT-ROS-TEST");
        mission.update(
                "MT-ROS-TEST",
                "ROS 主动状态测试",
                MissionType.COOPERATIVE_ESCORT,
                MissionExecutionMode.ROS_GAZEBO,
                "ESCORT_GUARD",
                "1.0.0",
                MissionStatus.READY,
                MissionStage.PREPARE,
                1,
                "护航目标",
                "匀速航行",
                "测试海域",
                null,
                null,
                "ROS 主动状态同步测试"
        );
        return new Fixture(
                new RosMissionStateSynchronizer(
                        runRepository, taskRepository, eventRepository),
                runRepository,
                taskRepository,
                eventRepository,
                mission
        );
    }

    private record Fixture(
            RosMissionStateSynchronizer synchronizer,
            MissionRunRepository runRepository,
            MissionTaskRepository taskRepository,
            MissionEventRepository eventRepository,
            MissionTask mission
    ) {
    }
}
