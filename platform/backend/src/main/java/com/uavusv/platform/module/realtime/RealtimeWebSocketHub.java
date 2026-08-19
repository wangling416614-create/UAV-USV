package com.uavusv.platform.module.realtime;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.ConcurrentWebSocketSessionDecorator;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.io.IOException;
import java.time.Clock;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Authenticated browser WebSocket fan-out for live ROS/Unity data.
 *
 * <p>The ROS-facing clients remain the only processes connected to the robot
 * network. Browsers subscribe to named topics here, so commands and telemetry
 * continue to pass through the platform's authorization and audit boundary.</p>
 */
@Component
public class RealtimeWebSocketHub extends TextWebSocketHandler {

    private static final Logger log = LoggerFactory.getLogger(RealtimeWebSocketHub.class);
    private static final int SEND_TIME_LIMIT_MILLIS = 2_000;
    private static final int BUFFER_SIZE_LIMIT_BYTES = 4 * 1024 * 1024;
    private static final int MESSAGE_SIZE_LIMIT_BYTES = 2 * 1024 * 1024;

    private final ObjectMapper objectMapper;
    private final Clock clock;
    private final Map<String, Client> clients = new ConcurrentHashMap<>();

    @Autowired
    public RealtimeWebSocketHub(ObjectMapper objectMapper) {
        this(objectMapper, Clock.systemUTC());
    }

    RealtimeWebSocketHub(ObjectMapper objectMapper, Clock clock) {
        this.objectMapper = objectMapper;
        this.clock = clock;
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        session.setTextMessageSizeLimit(MESSAGE_SIZE_LIMIT_BYTES);
        ConcurrentWebSocketSessionDecorator safeSession =
                new ConcurrentWebSocketSessionDecorator(
                        session,
                        SEND_TIME_LIMIT_MILLIS,
                        BUFFER_SIZE_LIMIT_BYTES
                );
        clients.put(session.getId(), new Client(safeSession, ConcurrentHashMap.newKeySet()));

        ObjectNode welcome = objectMapper.createObjectNode();
        welcome.put("type", "realtime_ready");
        welcome.put("timestampMs", clock.millis());
        safeSession.sendMessage(new TextMessage(welcome.toString()));
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) {
        Client client = clients.get(session.getId());
        if (client == null) return;
        try {
            JsonNode root = objectMapper.readTree(message.getPayload());
            if (!"subscribe".equals(root.path("action").asText())) return;
            Set<String> topics = new HashSet<>();
            root.path("topics").forEach(value -> {
                String topic = value.asText("").trim();
                if (!topic.isEmpty()) topics.add(topic);
            });
            client.topics().clear();
            client.topics().addAll(topics);
        } catch (Exception exception) {
            log.debug("Ignored invalid realtime subscription: {}", exception.getMessage());
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        clients.remove(session.getId());
    }

    @Override
    public void handleTransportError(WebSocketSession session, Throwable exception) {
        clients.remove(session.getId());
        try {
            session.close(CloseStatus.SERVER_ERROR);
        } catch (IOException ignored) {
            // The transport is already unusable.
        }
    }

    public void publish(String topic, JsonNode payload) {
        ObjectNode envelope = objectMapper.createObjectNode();
        envelope.put("type", "realtime_event");
        envelope.put("topic", topic);
        envelope.put("timestampMs", clock.millis());
        envelope.set("payload", payload);
        TextMessage message = new TextMessage(envelope.toString());

        clients.forEach((id, client) -> {
            if (!client.session().isOpen() || !client.accepts(topic)) return;
            try {
                client.session().sendMessage(message);
            } catch (Exception exception) {
                clients.remove(id);
                try {
                    client.session().close(CloseStatus.SERVER_ERROR);
                } catch (IOException ignored) {
                    // Best-effort cleanup only.
                }
            }
        });
    }

    int clientCount() {
        return clients.size();
    }

    private record Client(WebSocketSession session, Set<String> topics) {
        boolean accepts(String topic) {
            return topics.isEmpty() || topics.contains("*") || topics.contains(topic);
        }
    }
}
