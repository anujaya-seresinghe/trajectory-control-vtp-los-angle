import mqtt from 'mqtt';

// Topics published by ground_station/bridge (see bridge.hpp)
const TOPIC_PREFIX = 'uav';
const TOPIC_BRIDGE_STATUS = 'bridge/status';
const TOPIC_RAW_TX = 'mavlink/tx';

const TRAIL_MAX_POINTS = 20000;
const TRAIL_MIN_STEP = 0.05; // [m] only store a new trail point after moving this far
const RATE_WINDOW_MS = 2000;

export class Vehicle {
  constructor(sysid) {
    this.sysid = sysid;
    this.position = null; // {x, y, z, vx, vy, vz, time_boot_ms} in NED
    this.attitude = null; // {roll, pitch, yaw, ...} rad
    this.heartbeat = null;
    this.global = null; // {lat, lon, alt, relative_alt} from GLOBAL_POSITION_INT
    this.extState = null; // {vtol_state, landed_state} from EXTENDED_SYS_STATE
    this.modes = new Map(); // PX4 custom_mode -> {name, index, number_modes, standard_mode, properties}
    this.modesSeq = null; // AVAILABLE_MODES_MONITOR sequence
    this.params = new Map(); // PX4 parameter name -> {value, type}
    this.rosParams = new Map(); // ROS 2 fixed-wing mode parameters (PX4-style names) -> {value, type}
    this.rosGuidance = null; // latest ros_guidance from the ROS 2 fixed-wing mode: {state, r, index, points, a_m, ...}
    this.rosGuidanceAt = 0;
    this.lastPositionAt = 0;
    this.lastHeartbeatAt = 0;
    this.trail = []; // [[x, y, z], ...] NED
    this.trailVersion = 0; // bumped on every change so views can update lazily
    this._positionTimes = [];
  }

  get positionRateHz() {
    const now = performance.now();
    while (this._positionTimes.length && now - this._positionTimes[0] > RATE_WINDOW_MS) {
      this._positionTimes.shift();
    }
    return (this._positionTimes.length * 1000) / RATE_WINDOW_MS;
  }

  updatePosition(p) {
    this.position = p;
    this.lastPositionAt = performance.now();
    this._positionTimes.push(this.lastPositionAt);

    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(p.x - last[0], p.y - last[1], p.z - last[2]) >= TRAIL_MIN_STEP) {
      this.trail.push([p.x, p.y, p.z]);
      if (this.trail.length > TRAIL_MAX_POINTS) this.trail.splice(0, this.trail.length - TRAIL_MAX_POINTS);
      this.trailVersion++;
    }
  }

  clearTrail() {
    this.trail = this.position ? [[this.position.x, this.position.y, this.position.z]] : [];
    this.trailVersion++;
  }
}

/**
 * Subscribes to the bridge's telemetry topics and keeps one Vehicle per MAVLink system id.
 * Emits 'broker', 'bridge', 'vehicle-added' events.
 */
export class Telemetry extends EventTarget {
  constructor() {
    super();
    this.client = null;
    this.vehicles = new Map();
    this.brokerConnected = false;
    this.bridgeOnline = false;
  }

  connect(url) {
    this.disconnect();

    this.client = mqtt.connect(url, {
      clientId: `uav-gs-${Math.random().toString(16).slice(2, 10)}`,
      reconnectPeriod: 2000,
      connectTimeout: 5000,
      clean: true,
    });

    this.client.on('connect', () => {
      this._setBroker(true);
      this.client.subscribe([`${TOPIC_PREFIX}/+/+`, TOPIC_BRIDGE_STATUS]);
    });
    this.client.on('close', () => this._setBroker(false));
    this.client.on('offline', () => this._setBroker(false));
    this.client.on('error', (err) => console.warn('MQTT error:', err.message));
    this.client.on('message', (topic, payload) => this._onMessage(topic, payload));
  }

  disconnect() {
    if (this.client) {
      this.client.end(true);
      this.client = null;
    }
    this._setBroker(false);
    this._setBridge(false);
  }

  /**
   * Ask the bridge to upload a trajectory: it sends TRAJECTORY_SETPOINT_INITIATE and then one
   * TRAJECTORY_SETPOINT_UPLOAD per point, indexed in array order.
   * points: [{x, y, vx, vy, at, jt, t}, ...]
   */
  sendTrajectory(sysid, id, points) {
    if (!this.client?.connected) return false;
    const round = (v) => Math.round(v * 1e5) / 1e5;
    const payload = JSON.stringify({
      id,
      points: points.map((p) => [p.x, p.y, p.vx, p.vy, p.at, p.jt, p.t].map(round)),
    });
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/trajectory`, payload, { qos: 1 });
    return true;
  }

  /**
   * Ask the bridge to send MAV_CMD_DO_SET_MODE with a PX4 custom mode (main/sub mode).
   * The vehicle's COMMAND_ACK comes back as a 'command-ack' event.
   */
  sendSetMode(sysid, mainMode, subMode = 0) {
    if (!this.client?.connected) return false;
    const payload = JSON.stringify({ main_mode: mainMode, sub_mode: subMode });
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/set_mode`, payload, { qos: 1 });
    return true;
  }

  /** Send any COMMAND_LONG (params: up to 7 numbers, null = NaN); the ACK arrives as a 'command-ack' event. */
  sendCommand(sysid, command, params = []) {
    if (!this.client?.connected) return false;
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/command_long`, JSON.stringify({ command, params }), { qos: 1 });
    return true;
  }

  /** Send a COMMAND_INT: params p1..p4 (null = NaN), x/y integers (degE7 in global frames), z float. */
  sendCommandInt(sysid, command, frame, params, x, y, z = null) {
    if (!this.client?.connected) return false;
    const payload = JSON.stringify({ command, frame, params, x, y, z });
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/command_int`, payload, { qos: 1 });
    return true;
  }

  /** Ask the vehicle for the current value of each named parameter (answers arrive as 'param' events). */
  requestParams(sysid, names) {
    if (!this.client?.connected) return false;
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/param_get`, JSON.stringify({ names }), { qos: 1 });
    return true;
  }

  /** Set a float parameter; the vehicle confirms with the stored value as a 'param' event. */
  setParam(sysid, name, value) {
    if (!this.client?.connected) return false;
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/param_set`, JSON.stringify({ name, value }), { qos: 1 });
    return true;
  }

  /** VTOL: read parameters of the ROS 2 fixed-wing Trajectory mode (answers arrive as 'ros-param' events). */
  requestRosParams(sysid, names) {
    if (!this.client?.connected) return false;
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/ros_param_get`, JSON.stringify({ names }), { qos: 1 });
    return true;
  }

  /** VTOL: set a parameter of the ROS 2 mode; it answers with the stored value (and saves it to its params.yaml). */
  setRosParam(sysid, name, value) {
    if (!this.client?.connected) return false;
    this.client.publish(`${TOPIC_PREFIX}/${sysid}/cmd/ros_param_set`, JSON.stringify({ name, value }), { qos: 1 });
    return true;
  }

  /** Send a raw, already-packed MAVLink frame to the vehicle through the bridge. */
  sendRawMavlink(bytes) {
    this.client?.publish(TOPIC_RAW_TX, bytes);
  }

  _onMessage(topic, payload) {
    if (topic === TOPIC_BRIDGE_STATUS) {
      try {
        this._setBridge(Boolean(JSON.parse(payload.toString()).online));
      } catch {
        /* ignore malformed status */
      }
      return;
    }

    const [prefix, sysidStr, kind] = topic.split('/');
    if (prefix !== TOPIC_PREFIX) return;
    const sysid = Number(sysidStr);
    if (!Number.isInteger(sysid)) return;

    let data;
    try {
      data = JSON.parse(payload.toString());
    } catch {
      return;
    }

    const vehicle = this._vehicle(sysid);
    switch (kind) {
      case 'local_position_ned':
        vehicle.updatePosition(data);
        break;
      case 'attitude':
        vehicle.attitude = data;
        break;
      case 'heartbeat':
        vehicle.heartbeat = data;
        vehicle.lastHeartbeatAt = performance.now();
        break;
      case 'param':
        vehicle.params.set(data.name, { value: data.value, type: data.type });
        this.dispatchEvent(new CustomEvent('param', { detail: { sysid, ...data } }));
        break;
      case 'global_position':
        vehicle.global = data;
        break;
      case 'extended_sys_state':
        vehicle.extState = data;
        break;
      case 'available_mode':
        vehicle.modes.set(data.custom_mode, data);
        this.dispatchEvent(new CustomEvent('modes', { detail: vehicle }));
        break;
      case 'available_modes_monitor':
        // The set of modes changed (e.g. the ROS 2 mode registered): forget the old list and ask again
        if (vehicle.modesSeq !== data.seq) {
          const first = vehicle.modesSeq === null;
          vehicle.modesSeq = data.seq;
          if (!first) vehicle.modes.clear();
          this.dispatchEvent(new CustomEvent('modes-changed', { detail: vehicle }));
        }
        break;
      case 'ros_param':
        vehicle.rosParams.set(data.name, { value: data.value, type: data.type });
        this.dispatchEvent(new CustomEvent('ros-param', { detail: { sysid, ...data } }));
        break;
      case 'ros_guidance':
        vehicle.rosGuidance = data;
        vehicle.rosGuidanceAt = performance.now();
        break;
      case 'ros_trajectory_status':
        this.dispatchEvent(new CustomEvent('ros-trajectory-status', { detail: { sysid, ...data } }));
        break;
      case 'statustext':
        this.dispatchEvent(new CustomEvent('statustext', { detail: { sysid, ...data } }));
        break;
      case 'command_ack':
        this.dispatchEvent(new CustomEvent('command-ack', { detail: { sysid, ...data } }));
        break;
      case 'trajectory_status':
        this.dispatchEvent(new CustomEvent('trajectory-status', { detail: { sysid, ...data } }));
        break;
    }
  }

  _vehicle(sysid) {
    let v = this.vehicles.get(sysid);
    if (!v) {
      v = new Vehicle(sysid);
      this.vehicles.set(sysid, v);
      this.dispatchEvent(new CustomEvent('vehicle-added', { detail: v }));
    }
    return v;
  }

  _setBroker(connected) {
    if (connected === this.brokerConnected) return;
    this.brokerConnected = connected;
    if (!connected) this._setBridge(false);
    this.dispatchEvent(new CustomEvent('broker', { detail: connected }));
  }

  _setBridge(online) {
    if (online === this.bridgeOnline) return;
    this.bridgeOnline = online;
    this.dispatchEvent(new CustomEvent('bridge', { detail: online }));
  }
}
