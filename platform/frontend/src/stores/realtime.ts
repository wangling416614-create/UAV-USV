import { defineStore } from 'pinia'

import { useRadarSensorStore } from '@/stores/radarSensor'
import { useTrajectoryStore } from '@/stores/trajectory'
import { useVisualSensorStore } from '@/stores/visualSensor'
import type { RadarOverview } from '@/types/sensor'

type RealtimeStatus = 'IDLE' | 'CONNECTING' | 'ONLINE' | 'OFFLINE'

interface RealtimeState {
  status: RealtimeStatus
  lastMessageAt: number
  reconnectAttempts: number
  visualEnabled: boolean
  latestVisualDetections: Record<string, unknown>[]
  commandAcks: Record<string, RealtimeCommandAck>
  latestMissionStates: Record<string, Record<string, unknown>>
}

export interface RealtimeCommandAck {
  commandKey: string
  deviceCode: string
  status: number
  progress: number
  message: string
  receivedAt: number
}

interface RealtimeEnvelope {
  type: string
  topic?: string
  timestampMs?: number
  payload?: Record<string, unknown>
}

let socket: WebSocket | null = null
let reconnectTimer: number | undefined
let manualClose = false
const commandWaiters = new Map<string, Array<{
  resolve: (ack: RealtimeCommandAck) => void
  reject: (error: Error) => void
  timer: number
}>>()

function finite(value: unknown) {
  const parsed = Number(value)
  return Number.isFinite(parsed) ? parsed : 0
}

function yawDegrees(orientation: unknown) {
  if (!Array.isArray(orientation) || orientation.length < 4) return 0
  const x = finite(orientation[0])
  const y = finite(orientation[1])
  const z = finite(orientation[2])
  const w = finite(orientation[3])
  const siny = 2 * (w * z + x * y)
  const cosy = 1 - 2 * (y * y + z * z)
  return Math.atan2(siny, cosy) * 180 / Math.PI
}

function quaternion(orientation: unknown): [number, number, number, number] | null {
  if (!Array.isArray(orientation) || orientation.length < 4) return null
  const normalized: [number, number, number, number] = [
    finite(orientation[0]),
    finite(orientation[1]),
    finite(orientation[2]),
    finite(orientation[3]),
  ]
  return normalized.some(value => value !== 0) ? normalized : null
}

function trajectoryFromPose(payload: Record<string, unknown>) {
  const normalizeFleet = (items: unknown, type: 'UAV' | 'USV') => (
    Array.isArray(items) ? items : []
  ).map((raw) => {
    const item = raw as Record<string, unknown>
    const position = Array.isArray(item.position) ? item.position : []
    const status = item.status as Record<string, unknown> | undefined
    return {
      code: String(item.id ?? '').replace(/_/g, '-'),
      type,
      x: finite(position[0]),
      y: finite(position[1]),
      z: finite(position[2]),
      yaw: yawDegrees(item.orientation),
      orientation: quaternion(item.orientation),
      state: String(status?.mode ?? status?.status_text ?? 'ONLINE'),
    }
  }).filter(item => item.code)
  const mission = payload.mission as Record<string, unknown> | undefined
  const escort = mission?.escort as Record<string, unknown> | undefined
  const escortActive = escort?.active === true
  const target = (escortActive ? payload.friendly_ship : payload.target) as Record<string, unknown> | undefined
  const targetPosition = Array.isArray(target?.position) ? target.position : []
  const agents: Array<Record<string, unknown>> = [
    ...normalizeFleet(payload.uavs, 'UAV'),
    ...normalizeFleet(payload.usvs, 'USV'),
  ]
  if (target && targetPosition.length >= 2) {
    agents.push({
      code: String(target.id ?? 'target').replace(/_/g, '-'),
      type: 'TARGET',
      x: finite(targetPosition[0]),
      y: finite(targetPosition[1]),
      z: finite(targetPosition[2]),
      yaw: yawDegrees(target.orientation),
      orientation: quaternion(target.orientation),
      state: escortActive ? 'ESCORT_PROTECTED' : 'TRACKED',
    })
  }
  const capture = mission?.capture as Record<string, unknown> | undefined
  const roles = mission?.roles as Record<string, unknown> | undefined
  return {
    sequence: finite(payload.sequence),
    source: 'ros-gazebo',
    coordinateSystem: 'ROS_ENU',
    mission: {
      phase: String(escortActive
        ? (escort?.phase ?? 'ROS 护航态势')
        : (capture?.state_name ?? 'ROS 实时态势')),
      elapsed: 0,
      captureRadius: Math.max(0.1, finite(roles?.capture_radius) || 18),
      defenseRadius: 52,
      captureReady: escortActive ? !escort?.paused : capture?.degraded !== true,
      formationHolding: escortActive
        ? String(escort?.phase ?? '').toUpperCase() === 'ESCORTING'
        : String(capture?.state_name ?? '').toUpperCase() === 'HOLDING',
    },
    agents,
  } as Record<string, unknown>
}

function realtimeUrl() {
  const configured = String(import.meta.env.VITE_REALTIME_WS_URL ?? '').trim()
  if (configured) return configured
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${protocol}//${window.location.host}/api/v1/realtime`
}

export const useRealtimeStore = defineStore('realtime', {
  state: (): RealtimeState => ({
    status: 'IDLE',
    lastMessageAt: 0,
    reconnectAttempts: 0,
    visualEnabled: false,
    latestVisualDetections: [],
    commandAcks: {},
    latestMissionStates: {},
  }),
  actions: {
    connect() {
      manualClose = false
      if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) return
      if (reconnectTimer) window.clearTimeout(reconnectTimer)
      this.status = 'CONNECTING'
      const current = new WebSocket(realtimeUrl())
      socket = current
      current.onopen = () => {
        if (socket !== current) return
        this.status = 'ONLINE'
        this.reconnectAttempts = 0
        this.syncSubscription()
      }
      current.onmessage = (event) => {
        if (socket !== current || typeof event.data !== 'string') return
        try {
          const envelope = JSON.parse(event.data) as RealtimeEnvelope
          if (envelope.type !== 'realtime_event' || !envelope.topic || !envelope.payload) return
          this.lastMessageAt = Date.now()
          if (envelope.topic === 'radar.overview') {
            useRadarSensorStore().ingestOverview(envelope.payload as unknown as RadarOverview)
          } else if (envelope.topic === 'fleet.pose') {
            const frame = trajectoryFromPose(envelope.payload)
            const trajectoryStore = useTrajectoryStore()
            trajectoryStore.ingest(frame)
            trajectoryStore.ingestFor('MISSION_CENTER', frame)
          } else if (envelope.topic === 'visual.camera') {
            useVisualSensorStore().ingestRosFrame(envelope.payload)
          } else if (envelope.topic === 'visual.detections') {
            const detections = envelope.payload.detections
            this.latestVisualDetections = Array.isArray(detections) ? detections : []
          } else if (envelope.topic === 'control.ack') {
            this.ingestCommandAck(envelope.payload)
          } else if (envelope.topic === 'mission.state') {
            const algorithmCode = String(envelope.payload.algorithmCode ?? '')
            if (algorithmCode) {
              this.latestMissionStates = {
                ...this.latestMissionStates,
                [algorithmCode]: envelope.payload,
              }
            }
          }
        } catch {
          // A malformed event must not interrupt the live stream.
        }
      }
      current.onerror = () => current.close()
      current.onclose = () => {
        if (socket !== current) return
        socket = null
        this.status = 'OFFLINE'
        if (!manualClose) this.scheduleReconnect()
      }
    },
    scheduleReconnect() {
      if (manualClose || reconnectTimer) return
      this.reconnectAttempts += 1
      const delay = Math.min(10_000, 750 * 2 ** Math.min(this.reconnectAttempts - 1, 4))
      reconnectTimer = window.setTimeout(() => {
        reconnectTimer = undefined
        this.connect()
      }, delay)
    },
    syncSubscription() {
      if (!socket || socket.readyState !== WebSocket.OPEN) return
      const topics = ['fleet.pose', 'radar.overview', 'control.ack', 'mission.state']
      if (this.visualEnabled) topics.push('visual.camera', 'visual.detections')
      socket.send(JSON.stringify({ action: 'subscribe', topics }))
    },
    setVisualEnabled(enabled: boolean) {
      if (this.visualEnabled === enabled) return
      this.visualEnabled = enabled
      this.syncSubscription()
    },
    ingestCommandAck(payload: Record<string, unknown>) {
      const commandKey = String(payload.commandKey ?? payload.command_id ?? '')
      if (!commandKey) return
      const ack: RealtimeCommandAck = {
        commandKey,
        deviceCode: String(payload.deviceCode ?? payload.vehicle_id ?? ''),
        status: finite(payload.status),
        progress: finite(payload.progress),
        message: String(payload.message ?? ''),
        receivedAt: Date.now(),
      }
      this.commandAcks = { ...this.commandAcks, [commandKey]: ack }
      if (![1, 3, 4, 5, 6].includes(ack.status)) return
      const waiters = commandWaiters.get(commandKey) ?? []
      commandWaiters.delete(commandKey)
      waiters.forEach((waiter) => {
        window.clearTimeout(waiter.timer)
        if (ack.status === 1 || ack.status === 3) waiter.resolve(ack)
        else waiter.reject(new Error(ack.message || `ROS 指令失败，状态 ${ack.status}`))
      })
    },
    waitForCommandAck(commandKey: string, timeoutMs = 15_000): Promise<RealtimeCommandAck> {
      const current = this.commandAcks[commandKey]
      if (current && (current.status === 1 || current.status === 3)) return Promise.resolve(current)
      if (current && [4, 5, 6].includes(current.status)) {
        return Promise.reject(new Error(current.message || `ROS 指令失败，状态 ${current.status}`))
      }
      return new Promise((resolve, reject) => {
        const timer = window.setTimeout(() => {
          const pending = commandWaiters.get(commandKey) ?? []
          commandWaiters.set(commandKey, pending.filter(item => item.timer !== timer))
          reject(new Error(`等待 ROS 指令确认超时：${commandKey}`))
        }, timeoutMs)
        const pending = commandWaiters.get(commandKey) ?? []
        pending.push({ resolve, reject, timer })
        commandWaiters.set(commandKey, pending)
      })
    },
    disconnect() {
      manualClose = true
      if (reconnectTimer) window.clearTimeout(reconnectTimer)
      reconnectTimer = undefined
      const current = socket
      socket = null
      if (current && current.readyState < WebSocket.CLOSING) current.close(1000, 'leaving authenticated console')
      this.status = 'IDLE'
      commandWaiters.forEach(waiters => waiters.forEach((waiter) => {
        window.clearTimeout(waiter.timer)
        waiter.reject(new Error('实时链路已断开'))
      }))
      commandWaiters.clear()
    },
  },
})
