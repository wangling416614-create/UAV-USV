package com.uavusv.platform.module.mission.event;

public record RosMissionStateObservedEvent(
        String algorithmCode,
        String status,
        String phase,
        String targetId,
        String reason,
        long timestampMs,
        String confirmedCommandKey,
        String confirmedCommandType
) {
}
