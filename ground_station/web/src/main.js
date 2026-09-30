import { Telemetry } from './telemetry.js';
import {
  px4ModeName,
  mavResultName,
  isTrajectoryMode,
  isVtol,
  isExternalMode,
  rosTrajectoryMode,
  vtolStateName,
  MAV_VTOL_STATE,
  PX4_MAIN_MODE_AUTO,
  PX4_SUB_MODE_AUTO_TRAJ,
  PX4_SUB_MODE_AUTO_LOITER,
  TRAJ_PARAMS,
} from './px4.js';
import { View2D } from './view2d.js';
import { TrajectoryPlan, DEFAULT_SPEED, SAMPLE_SPACING } from './trajectory.js';

const STALE_MS = 3000;
const BROKER_KEY = 'uav-gs.broker';
const VIEW_KEY = 'uav-gs.view';

const $ = (id) => document.getElementById(id);
const deg = (rad) => (rad * 180) / Math.PI;
const fmt = (v, digits = 2) => (Number.isFinite(v) ? v.toFixed(digits) : '—');

function storageGet(key) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}
function storageSet(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch {
    /* storage unavailable */
  }
}

// ---------------------------------------------------------------- telemetry

const telemetry = new Telemetry();
let selectedId = null;

// VITE_MQTT_WS_PORT is set at build time by the Docker image (docker-compose-gcs.yaml)
const defaultBroker = `ws://${location.hostname || 'localhost'}:${import.meta.env.VITE_MQTT_WS_PORT || 9001}`;
const brokerInput = $('broker-url');
brokerInput.value = new URLSearchParams(location.search).get('broker') ?? storageGet(BROKER_KEY) ?? defaultBroker;

$('broker-form').addEventListener('submit', (ev) => {
  ev.preventDefault();
  const url = brokerInput.value.trim();
  storageSet(BROKER_KEY, url);
  telemetry.connect(url);
});

telemetry.addEventListener('broker', (ev) => setPill('st-broker', ev.detail ? 'ok' : 'off'));
telemetry.addEventListener('bridge', (ev) => setPill('st-bridge', ev.detail ? 'ok' : 'off'));
telemetry.addEventListener('vehicle-added', (ev) => {
  if (selectedId === null) selectedId = ev.detail.sysid;
  rebuildVehicleSelect();
});

telemetry.connect(brokerInput.value);

function setPill(id, state) {
  $(id).dataset.state = state;
}

const vehicleSelect = $('vehicle-select');
vehicleSelect.addEventListener('change', () => {
  selectedId = Number(vehicleSelect.value);
  activeView.recenter(selectedVehicle());
});

function rebuildVehicleSelect() {
  vehicleSelect.replaceChildren(
    ...[...telemetry.vehicles.keys()].sort((a, b) => a - b).map((id) => new Option(`System ${id}`, id)),
  );
  vehicleSelect.value = selectedId;
  vehicleSelect.disabled = false;
}

function selectedVehicle() {
  return telemetry.vehicles.get(selectedId) ?? null;
}

// ---------------------------------------------------------------- views

const views = { '2d': new View2D($('view-2d')), '3d': null };

// ?debug: hooks for scripts/record_gcs_gifs.py (map geometry, selected vehicle, ROS 2 mode availability)
if (new URLSearchParams(location.search).has('debug')) {
  window.__gcs = { view2d: views['2d'], selected: () => selectedVehicle(), rosMode: () => rosTrajectoryMode(selectedVehicle()) };
}
let activeName = '2d';
let activeView = views['2d'];
const followBox = $('follow');

async function setView(name) {
  if (name === '3d' && planner.active) return; // planning is 2D only
  if (name === '3d' && !views['3d']) {
    // three.js is only loaded the first time the 3D view is opened
    const { View3D } = await import('./view3d.js');
    views['3d'] = new View3D($('view-3d'));
    views['3d'].recenter(selectedVehicle());
  }

  activeName = name;
  activeView = views[name];
  activeView.setFollow(followBox.checked);
  $('view-2d').hidden = name !== '2d';
  $('view-3d').hidden = name !== '3d';
  activeView.resize();
  $('view-hint').textContent =
    name === '2d' ? 'Drag to pan · scroll to zoom' : 'Drag to orbit · right-drag to pan · scroll to zoom';

  for (const btn of document.querySelectorAll('[data-view]')) {
    btn.setAttribute('aria-selected', String(btn.dataset.view === name));
  }
  storageSet(VIEW_KEY, name);
  updatePlannerAvailability();
}

for (const btn of document.querySelectorAll('[data-view]')) {
  btn.addEventListener('click', () => setView(btn.dataset.view));
}

followBox.addEventListener('change', () => activeView.setFollow(followBox.checked));
// Panning the 2D map by hand turns follow off
$('view-2d').addEventListener('user-pan', () => {
  followBox.checked = false;
  activeView.setFollow(false);
});
$('recenter').addEventListener('click', () => activeView.recenter(selectedVehicle()));
$('clear-trail').addEventListener('click', () => {
  for (const v of telemetry.vehicles.values()) v.clearTrail();
});

// ---------------------------------------------------------------- commands with ACK

const MAV_CMD_NAV_TAKEOFF = 22;
const MAV_CMD_DO_REPOSITION = 192;
const MAV_FRAME_GLOBAL = 0;
const MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1;
const MAV_CMD_COMPONENT_ARM_DISARM = 400;
const MAV_CMD_REQUEST_MESSAGE = 512;
const MAV_CMD_DO_VTOL_TRANSITION = 3000;
const MAVLINK_MSG_ID_AVAILABLE_MODES = 435;
const COMMAND_TIMEOUT_MS = 3000;
const pendingCommands = new Map(); // command id -> { resolve, timer }

/**
 * Send a command and resolve with its MAV_RESULT, or null if no ACK arrives in time.
 * `send` defaults to COMMAND_LONG with `params`; pass a function to send it differently (COMMAND_INT).
 */
function commandWithAck(sysid, command, params, send = () => telemetry.sendCommand(sysid, command, params)) {
  pendingCommands.get(command)?.resolve(null);
  send();
  return new Promise((resolve) => {
    const timer = setTimeout(() => {
      pendingCommands.delete(command);
      resolve(null);
    }, COMMAND_TIMEOUT_MS);
    pendingCommands.set(command, {
      resolve: (result) => {
        clearTimeout(timer);
        pendingCommands.delete(command);
        resolve(result);
      },
    });
  });
}

telemetry.addEventListener('command-ack', (ev) => {
  const { sysid, command, result } = ev.detail;
  if (sysid === selectedId) pendingCommands.get(command)?.resolve(result);
});

// PX4 explains rejections (e.g. failed preflight checks) in STATUSTEXT
let lastStatusText = null; // { text, severity, at }
telemetry.addEventListener('statustext', (ev) => {
  if (ev.detail.sysid === selectedId) lastStatusText = { ...ev.detail, at: performance.now() };
});
function statusTextSince(t0) {
  return lastStatusText && lastStatusText.at >= t0 ? ` PX4: "${lastStatusText.text}"` : '';
}

// ---------------------------------------------------------------- takeoff

const takeoffBtn = $('takeoff-btn');
const takeoffAlt = $('takeoff-alt');
const takeoffStatus = $('takeoff-status');
const TAKEOFF_PARAM = 'MIS_TAKEOFF_ALT'; // PX4 climbs to current altitude + this when NAV_TAKEOFF has no altitude
const paramWaiters = []; // { name, resolve, timer }

takeoffAlt.addEventListener('input', () => takeoffAlt.removeAttribute('aria-invalid'));

/** Resolve with the value PX4 reports for `name` after a set, or null on timeout. */
function waitForParam(name) {
  return new Promise((resolve) => {
    const waiter = { name, resolve };
    waiter.timer = setTimeout(() => {
      paramWaiters.splice(paramWaiters.indexOf(waiter), 1);
      resolve(null);
    }, COMMAND_TIMEOUT_MS);
    paramWaiters.push(waiter);
  });
}

telemetry.addEventListener('param', (ev) => {
  const { sysid, name, value } = ev.detail;
  if (sysid !== selectedId) return;
  for (const w of paramWaiters.filter((w) => w.name === name)) {
    clearTimeout(w.timer);
    paramWaiters.splice(paramWaiters.indexOf(w), 1);
    w.resolve(value);
  }
});

const VTOL_TAKEOFF_ALT = 40; // [m] default for VTOLs: high enough for the transition
let takeoffAltTouched = false;
takeoffAlt.addEventListener('input', () => (takeoffAltTouched = true));

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const altitudeOf = (v) => (v?.position ? -v.position.z : NaN);

/** Poll `check` until it returns true; resolves false after `timeoutMs`. */
async function waitUntil(check, timeoutMs, onTick) {
  const t0 = performance.now();
  while (performance.now() - t0 < timeoutMs) {
    if (check()) return true;
    onTick?.();
    await sleep(250);
  }
  return false;
}

/** Validate the altitude field; returns the altitude or null (after reporting why). */
function takeoffAltitude(say) {
  const alt = Number(takeoffAlt.value);
  if (takeoffAlt.value.trim() === '' || !Number.isFinite(alt) || alt < 1 || alt > 100) {
    takeoffAlt.setAttribute('aria-invalid', 'true');
    say('Altitude must be between 1 and 100 m.');
    return null;
  }
  return alt;
}

/** MIS_TAKEOFF_ALT, arm, NAV_TAKEOFF. Returns true once PX4 accepted the takeoff. */
async function takeoff(vehicle, alt, say, t0) {
  // 1. Takeoff altitude: PX4 uses MIS_TAKEOFF_ALT above the current position
  const current = vehicle.params.get(TAKEOFF_PARAM)?.value;
  if (current === undefined || Math.abs(current - alt) > 1e-3) {
    say(`Setting ${TAKEOFF_PARAM} to ${alt} m…`);
    const waiting = waitForParam(TAKEOFF_PARAM);
    telemetry.setParam(vehicle.sysid, TAKEOFF_PARAM, alt);
    const stored = await waiting;
    if (stored === null || Math.abs(stored - alt) > 1e-3) {
      say(`Could not set ${TAKEOFF_PARAM} (vehicle reports ${stored ?? 'nothing'}).`);
      return false;
    }
  }

  // 2. Arm
  if (!vehicle.heartbeat?.armed) {
    say('Arming…');
    const result = await commandWithAck(vehicle.sysid, MAV_CMD_COMPONENT_ARM_DISARM, [1]);
    if (result !== 0) {
      say(`Arming ${result === null ? 'got no ACK' : mavResultName(result)}.${statusTextSince(t0)}`);
      return false;
    }
  }

  // 3. Take off (no altitude/position: current position + MIS_TAKEOFF_ALT)
  say('Taking off…');
  const result = await commandWithAck(vehicle.sysid, MAV_CMD_NAV_TAKEOFF, [null, null, null, null, null, null, null]);
  if (result !== 0) {
    say(`Takeoff ${result === null ? 'got no ACK' : mavResultName(result)}.${statusTextSince(t0)}`);
    return false;
  }
  return true;
}

function flightChecks(say) {
  const vehicle = selectedVehicle();
  if (!telemetry.brokerConnected) return say('Not connected to the MQTT broker.'), null;
  if (!telemetry.bridgeOnline) return say('The MAVLink bridge is offline.'), null;
  if (!vehicle) return say('No vehicle connected.'), null;
  return vehicle;
}

takeoffBtn.addEventListener('click', async () => {
  const say = (msg) => (takeoffStatus.textContent = msg);
  const vehicle = flightChecks(say);
  if (!vehicle) return;
  const alt = takeoffAltitude(say);
  if (alt === null) return;
  if (vehicle.heartbeat?.armed && altitudeOf(vehicle) > 1.0) {
    return say(`Already in the air (${altitudeOf(vehicle).toFixed(1)} m).`);
  }

  takeoffBtn.disabled = true;
  try {
    if (await takeoff(vehicle, alt, say, performance.now())) say(`Takeoff accepted: climbing ${alt} m.`);
  } finally {
    takeoffBtn.disabled = false;
  }
});

// VTOL only: transition an airborne multicopter to fixed-wing flight (take off first with the Takeoff button)
const fwBtn = $('fw-btn');
fwBtn.addEventListener('click', async () => {
  const say = (msg) => (takeoffStatus.textContent = msg);
  const vehicle = flightChecks(say);
  if (!vehicle) return;
  if (!isVtol(vehicle)) return say('This vehicle is not a VTOL.');
  if (vehicle.extState?.vtol_state === MAV_VTOL_STATE.FW) return say('Already in fixed-wing flight.');
  if (!vehicle.heartbeat?.armed || !(altitudeOf(vehicle) > 1.0)) return say('Take off first, then switch to fixed-wing.');
  // PX4 ignores a transition while Takeoff is still running
  if (((vehicle.heartbeat?.custom_mode >>> 24) & 0xff) === 2) return say('Still taking off; wait until the climb is done.');

  fwBtn.disabled = true;
  const t0 = performance.now();
  try {
    for (let attempt = 1; attempt <= 2; attempt++) {
      say('Transitioning to fixed-wing…');
      const result = await commandWithAck(vehicle.sysid, MAV_CMD_DO_VTOL_TRANSITION, [MAV_VTOL_STATE.FW, 0]);
      if (result !== 0) {
        return say(`Transition ${result === null ? 'got no ACK' : mavResultName(result)}.${statusTextSince(t0)}`);
      }
      const started = await waitUntil(() => vehicle.extState?.vtol_state !== MAV_VTOL_STATE.MC, 5000);
      if (started) break;
      if (attempt === 2) return say(`PX4 accepted the transition but stayed a multicopter.${statusTextSince(t0)}`);
    }

    const done = await waitUntil(
      () => vehicle.extState?.vtol_state === MAV_VTOL_STATE.FW,
      45000,
      () => say(`Transitioning… (${vtolStateName(vehicle)})`),
    );
    say(done ? 'Fixed-wing flight. The ROS 2 Trajectory mode registers now.' : `Transition did not finish.${statusTextSince(t0)}`);
  } finally {
    fwBtn.disabled = false;
  }
});

// ---------------------------------------------------------------- available modes (to find the ROS 2 mode)

function requestAvailableModes(vehicle) {
  if (vehicle && telemetry.bridgeOnline) {
    // param2 = 0: all modes; PX4 answers with one AVAILABLE_MODES per mode
    telemetry.sendCommand(vehicle.sysid, MAV_CMD_REQUEST_MESSAGE, [MAVLINK_MSG_ID_AVAILABLE_MODES, 0]);
  }
}
telemetry.addEventListener('vehicle-added', (ev) => requestAvailableModes(ev.detail));
telemetry.addEventListener('modes-changed', (ev) => requestAvailableModes(ev.detail));

/** Display name of the current mode, preferring PX4's own names (external modes are only known that way) */
function modeName(vehicle) {
  const custom = vehicle?.heartbeat?.custom_mode;
  if (custom === undefined) return '—';
  const listed = vehicle.modes.get(custom);
  if (listed && isExternalMode(custom)) return `${listed.name} (ROS 2)`;
  return px4ModeName(custom);
}

// ---------------------------------------------------------------- map popup: "Go here"

const EARTH_RADIUS = 6371000; // [m], PX4's CONSTANTS_RADIUS_OF_EARTH
const GOTO_REACHED_M = 1.5;
const mapPopup = $('map-popup');
const popupStatus = $('map-popup-status');
let popupTarget = null; // {n, e}
let gotoTarget = null; // active target {n, e}, drawn on the map
let popupCloseTimer = null;

function openMapPopup({ sx, sy, n, e }) {
  clearTimeout(popupCloseTimer);
  popupTarget = { n, e };
  $('map-popup-coords').textContent = `N ${n.toFixed(1)} m · E ${e.toFixed(1)} m`;
  popupStatus.textContent = '';
  $('goto-btn').disabled = false;
  mapPopup.hidden = false;
  // keep the popup inside the map
  const host = $('view-2d');
  const w = mapPopup.offsetWidth;
  const h = mapPopup.offsetHeight;
  mapPopup.style.left = `${Math.min(sx + 8, host.clientWidth - w - 8)}px`;
  mapPopup.style.top = `${Math.min(sy + 8, host.clientHeight - h - 8)}px`;
}

function closeMapPopup() {
  clearTimeout(popupCloseTimer);
  mapPopup.hidden = true;
  popupTarget = null;
}

/** Local NED (n, e) -> lat/lon, relative to the vehicle's current global and local position (flat earth). */
function localToGlobal(vehicle, n, e) {
  const { lat, lon } = vehicle.global;
  const dn = n - vehicle.position.x;
  const de = e - vehicle.position.y;
  const latRad = (lat * Math.PI) / 180;
  return {
    lat: lat + (dn / EARTH_RADIUS) * (180 / Math.PI),
    lon: lon + (de / (EARTH_RADIUS * Math.cos(latRad))) * (180 / Math.PI),
  };
}

$('view-2d').addEventListener('map-context', (ev) => openMapPopup(ev.detail));
$('view-2d').addEventListener('map-pointerdown', closeMapPopup); // map pan/zoom
// Any click outside the popup closes it (a right-click on the map reopens it at the new spot)
document.addEventListener('pointerdown', (ev) => {
  if (!mapPopup.hidden && !mapPopup.contains(ev.target)) closeMapPopup();
});
mapPopup.addEventListener('contextmenu', (ev) => ev.preventDefault());
document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && !mapPopup.hidden) closeMapPopup();
});

$('goto-btn').addEventListener('click', async () => {
  const target = popupTarget;
  const vehicle = selectedVehicle();
  const say = (msg) => (popupStatus.textContent = msg);
  if (!target) return;
  if (!telemetry.bridgeOnline) return say('The MAVLink bridge is offline.');
  if (!vehicle) return say('No vehicle connected.');
  if (!vehicle.heartbeat?.armed) return say('The UAV is not armed. Take off first.');
  if (!vehicle.global || !vehicle.position) return say('No global position from the vehicle yet.');

  const { lat, lon } = localToGlobal(vehicle, target.n, target.e);
  const t0 = performance.now();
  $('goto-btn').disabled = true;
  say('Sending…');
  // DO_REPOSITION: default speed, switch to Hold, default radius, keep yaw; lat/lon target, keep altitude (z = NaN)
  const params = [-1, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, null];
  const result = await commandWithAck(vehicle.sysid, MAV_CMD_DO_REPOSITION, params, () =>
    telemetry.sendCommandInt(
      vehicle.sysid,
      MAV_CMD_DO_REPOSITION,
      MAV_FRAME_GLOBAL,
      params,
      Math.round(lat * 1e7),
      Math.round(lon * 1e7),
      null,
    ),
  );
  if (result === 0) {
    gotoTarget = target;
    views['2d'].setGotoTarget(target);
    say('Going there (Hold mode).');
    popupCloseTimer = setTimeout(closeMapPopup, 1200);
  } else {
    $('goto-btn').disabled = false;
    say(`${result === null ? 'No ACK' : `Rejected (${mavResultName(result)})`}.${statusTextSince(t0)}`);
  }
});

/** Clear the target marker once reached, or when the UAV leaves Hold (e.g. back to Trajectory). */
function updateGotoTarget(vehicle) {
  if (!gotoTarget || !vehicle?.position) return;
  const reached = Math.hypot(vehicle.position.x - gotoTarget.n, vehicle.position.y - gotoTarget.e) < GOTO_REACHED_M;
  const custom = vehicle.heartbeat?.custom_mode ?? 0;
  const inHold = ((custom >>> 16) & 0xff) === PX4_MAIN_MODE_AUTO && ((custom >>> 24) & 0xff) === PX4_SUB_MODE_AUTO_LOITER;
  if (reached || !inHold) {
    gotoTarget = null;
    views['2d'].setGotoTarget(null);
  }
}

// ---------------------------------------------------------------- trajectory flight mode

const MAV_CMD_DO_SET_MODE = 176;
const ACK_TIMEOUT_MS = 3000;
const modeStatus = $('mode-status');
let pendingModeRequest = null; // { resolve, timer }

/**
 * Send MAV_CMD_DO_SET_MODE (PX4 AUTO / subMode) and wait for the COMMAND_ACK.
 * Resolves true if the vehicle accepted; the status line under the buttons explains any failure.
 */
function requestMode(subMode, label) {
  const vehicle = selectedVehicle();
  if (!telemetry.brokerConnected) return fail('Not connected to the MQTT broker.');
  if (!telemetry.bridgeOnline) return fail('The MAVLink bridge is offline.');
  if (!vehicle) return fail('No vehicle connected.');

  finishModeRequest(false); // a newer request replaces an unanswered one
  telemetry.sendSetMode(vehicle.sysid, PX4_MAIN_MODE_AUTO, subMode);
  modeStatus.textContent = `Requesting ${label} mode on system ${vehicle.sysid}…`;
  return new Promise((resolve) => {
    pendingModeRequest = {
      label,
      resolve,
      timer: setTimeout(() => {
        modeStatus.textContent = 'No ACK received from the vehicle.';
        finishModeRequest(false);
      }, ACK_TIMEOUT_MS),
    };
  });

  function fail(message) {
    modeStatus.textContent = message;
    return Promise.resolve(false);
  }
}

/**
 * Trajectory mode for the selected vehicle: the PX4-internal mode for multicopters, the ROS 2 external
 * mode (ros2/src/trajectory_mode_fw) for VTOLs, which registers itself once in fixed-wing flight.
 */
function requestTrajectoryMode() {
  const vehicle = selectedVehicle();
  if (!isVtol(vehicle)) return requestMode(PX4_SUB_MODE_AUTO_TRAJ, 'Trajectory');

  const custom = rosTrajectoryMode(vehicle);
  if (custom === null) {
    requestAvailableModes(vehicle);
    modeStatus.textContent =
      vehicle.extState?.vtol_state === MAV_VTOL_STATE.FW
        ? 'The ROS 2 Trajectory mode is not registered. Is trajectory_mode_fw running?'
        : 'VTOL: the ROS 2 Trajectory mode is only available in fixed-wing flight. Take off, then press Fixed-wing.';
    return Promise.resolve(false);
  }
  return requestMode((custom >>> 24) & 0xff, 'Trajectory (ROS 2)');
}

/** True if the vehicle flies the trajectory mode that matches its type */
function inTrajectoryMode(vehicle) {
  const custom = vehicle?.heartbeat?.custom_mode;
  if (custom === undefined) return false;
  return isVtol(vehicle) ? custom === rosTrajectoryMode(vehicle) : isTrajectoryMode(custom);
}

function finishModeRequest(accepted) {
  if (!pendingModeRequest) return;
  clearTimeout(pendingModeRequest.timer);
  pendingModeRequest.resolve(accepted);
  pendingModeRequest = null;
}

$('hold-btn').addEventListener('click', () => requestMode(PX4_SUB_MODE_AUTO_LOITER, 'Hold'));
$('traj-mode-btn').addEventListener('click', () => requestTrajectoryMode());

telemetry.addEventListener('command-ack', (ev) => {
  const { sysid, command, result } = ev.detail;
  if (command !== MAV_CMD_DO_SET_MODE || !pendingModeRequest) return;
  const { label } = pendingModeRequest;
  modeStatus.textContent =
    result === 0
      ? `${label} mode accepted by system ${sysid}.`
      : `${label} mode ${mavResultName(result)} by system ${sysid}.`;
  finishModeRequest(result === 0);
});

// ---------------------------------------------------------------- controller parameters

const PARAM_TIMEOUT_MS = 3000;
const paramsStatus = $('params-status');
const paramInputs = new Map(); // name -> input element
const pendingParams = new Map(); // name -> { value, timer }
let paramBatch = []; // results of the last Apply, shown together
let paramsRequestedFor = null; // sysid whose values were last requested
// Where the guidance parameters live: 'px4' (internal multicopter mode, TRAJ_* PX4 parameters over MAVLink)
// or 'ros' (VTOL: the ROS 2 fixed-wing mode's parameters, MAVLink to its own component, saved to its params.yaml)
let paramSource = 'px4';
const paramSourceFor = (vehicle) => (isVtol(vehicle) ? 'ros' : 'px4');
const paramStore = (vehicle) => (paramSource === 'ros' ? vehicle?.rosParams : vehicle?.params);

for (const def of TRAJ_PARAMS) {
  const row = document.createElement('label');
  row.className = 'param-row';
  row.innerHTML = `<span class="sym"></span><span class="pname"></span>`;
  row.querySelector('.sym').textContent = def.label;
  row.querySelector('.pname').textContent = def.unit ? `${def.name} [${def.unit}]` : def.name;
  const input = document.createElement('input');
  Object.assign(input, { type: 'number', step: String(def.step), min: String(def.min), max: String(def.max) });
  input.inputMode = 'decimal';
  input.placeholder = '—';
  input.addEventListener('input', () => {
    input.dataset.state = 'dirty';
    input.removeAttribute('aria-invalid');
  });
  row.append(input);
  $('param-rows').append(row);
  paramInputs.set(def.name, input);
}

function refreshParams() {
  const vehicle = selectedVehicle();
  if (!vehicle || !telemetry.bridgeOnline) return;
  paramsRequestedFor = vehicle.sysid;
  const names = TRAJ_PARAMS.map((d) => d.name);
  if (paramSource === 'ros') {
    telemetry.requestParams(vehicle.sysid, [TAKEOFF_PARAM]); // the takeoff altitude is still a PX4 parameter
    telemetry.requestRosParams(vehicle.sysid, names);
  } else {
    telemetry.requestParams(vehicle.sysid, [...names, TAKEOFF_PARAM]);
  }
  paramsStatus.textContent = 'Reading parameters…';
  setTimeout(() => {
    const missing = TRAJ_PARAMS.filter((d) => !paramStore(selectedVehicle())?.has(d.name)).map((d) => d.name);
    if (missing.length) {
      paramsStatus.textContent =
        paramSource === 'ros'
          ? 'No answer from the ROS 2 fixed-wing mode. Is trajectory_mode_fw running?'
          : `No value from the vehicle for ${missing.join(', ')}. Is the PX4 build up to date?`;
    }
  }, PARAM_TIMEOUT_MS);
}

function showParam(name, value) {
  const input = paramInputs.get(name);
  if (!input || input.dataset.state === 'dirty') return; // don't overwrite the user's unsent edit
  input.value = String(+Number(value).toPrecision(6));
  input.dataset.state = '';
}

$('params-refresh').addEventListener('click', () => {
  for (const input of paramInputs.values()) input.dataset.state = '';
  refreshParams();
});

$('params-form').addEventListener('submit', (ev) => {
  ev.preventDefault();
  const vehicle = selectedVehicle();
  if (!telemetry.bridgeOnline || !vehicle) {
    paramsStatus.textContent = 'No vehicle / bridge connection.';
    return;
  }

  const changes = [];
  for (const def of TRAJ_PARAMS) {
    const input = paramInputs.get(def.name);
    if (input.dataset.state !== 'dirty') continue;
    const value = Number(input.value);
    if (input.value.trim() === '' || !Number.isFinite(value) || value < def.min || value > def.max) {
      input.setAttribute('aria-invalid', 'true');
      paramsStatus.textContent = `${def.name} must be between ${def.min} and ${def.max}.`;
      input.focus();
      return;
    }
    changes.push({ name: def.name, value, input });
  }
  if (!changes.length) {
    paramsStatus.textContent = 'Nothing changed.';
    return;
  }

  paramBatch = [];
  for (const { name, value, input } of changes) {
    clearTimeout(pendingParams.get(name)?.timer);
    input.dataset.state = 'pending';
    if (paramSource === 'ros') telemetry.setRosParam(vehicle.sysid, name, value);
    else telemetry.setParam(vehicle.sysid, name, value);
    pendingParams.set(name, {
      value,
      timer: setTimeout(() => {
        pendingParams.delete(name);
        input.dataset.state = 'dirty';
        paramBatch.push(`no confirmation for ${name}`);
        paramsStatus.textContent = paramBatch.join(' · ');
      }, PARAM_TIMEOUT_MS),
    });
  }
  paramsStatus.textContent = `Setting ${changes.map((c) => c.name).join(', ')}…`;
});

telemetry.addEventListener('param', (ev) => {
  const { sysid, name, value } = ev.detail;
  if (paramSource !== 'px4') return; // VTOL: the panel shows the ROS 2 mode's values
  onParamValue(sysid, name, value);
});

telemetry.addEventListener('ros-param', (ev) => {
  const { sysid, name, value, error } = ev.detail;
  if (paramSource !== 'ros') return;
  onParamValue(sysid, name, value, error);
});

function onParamValue(sysid, name, value, error) {
  if (sysid !== selectedId || !paramInputs.has(name)) return;

  const pending = pendingParams.get(name);
  if (pending) {
    clearTimeout(pending.timer);
    pendingParams.delete(name);
    const input = paramInputs.get(name);
    input.dataset.state = '';
    // float32 round trip: compare with a relative tolerance
    const ok = !error && Math.abs(value - pending.value) <= 1e-5 * Math.max(1, Math.abs(pending.value));
    paramBatch.push(
      ok
        ? `${name} = ${+Number(value).toPrecision(6)}${paramSource === 'ros' ? ' (saved to params.yaml)' : ''}`
        : `${name} stayed ${value} (rejected ${pending.value}${error ? `: ${error}` : ''})`,
    );
    paramsStatus.textContent = (pendingParams.size ? 'Setting… ' : 'Set: ') + paramBatch.join(' · ');
  } else if (paramsStatus.textContent === 'Reading parameters…') {
    paramsStatus.textContent = '';
  }
  showParam(name, value);
}

/** Switch the panel between PX4 (multicopter) and the ROS 2 mode (VTOL) when the vehicle type becomes known */
function updateParamSource(vehicle) {
  const source = paramSourceFor(vehicle);
  if (source === paramSource) return;
  paramSource = source;
  $('params-source').textContent = source === 'ros' ? '(ROS 2 fixed-wing mode)' : '(PX4)';
  for (const input of paramInputs.values()) {
    input.dataset.state = '';
    input.value = '';
  }
  for (const pending of pendingParams.values()) clearTimeout(pending.timer);
  pendingParams.clear();
  paramsRequestedFor = null;
  maybeRequestParams();
}

// Read the values whenever a vehicle becomes available or is selected
function maybeRequestParams() {
  const vehicle = selectedVehicle();
  if (vehicle && telemetry.bridgeOnline && paramsRequestedFor !== vehicle.sysid) refreshParams();
}
telemetry.addEventListener('vehicle-added', maybeRequestParams);
telemetry.addEventListener('bridge', (ev) => {
  if (ev.detail) {
    paramsRequestedFor = null;
    maybeRequestParams();
  }
});
vehicleSelect.addEventListener('change', () => {
  for (const input of paramInputs.values()) input.dataset.state = '';
  const v = selectedVehicle();
  updateParamSource(v);
  for (const def of TRAJ_PARAMS) {
    const p = paramStore(v)?.get(def.name);
    paramInputs.get(def.name).value = p ? String(+Number(p.value).toPrecision(6)) : '';
  }
  maybeRequestParams();
});

// ---------------------------------------------------------------- trajectory planner

const planner = { active: false, plan: null, renderedVersion: -1 };
const planBtn = $('plan-btn');
const planEditor = $('plan-editor');
const idInput = $('traj-id');
const planError = $('plan-error');
const uploadStatus = $('upload-status');
const view3dTab = document.querySelector('[data-view="3d"]');

function updatePlannerAvailability() {
  const in2d = activeName === '2d';
  planBtn.hidden = planner.active || !in2d;
  $('plan-unavailable').hidden = planner.active || in2d;
  view3dTab.disabled = planner.active;
  view3dTab.title = planner.active ? 'Finish or cancel the trajectory first' : '';
}

function startPlanning() {
  if (activeName !== '2d') return;
  planner.active = true;
  planner.plan = new TrajectoryPlan();
  planner.renderedVersion = -1;
  // A moving map makes placing points impossible
  followBox.checked = false;
  views['2d'].setFollow(false);
  views['2d'].setPlan(planner.plan);
  planEditor.hidden = false;
  planError.textContent = '';
  $('plan-hold-note').textContent = isVtol(selectedVehicle())
    ? 'The VTOL is in the ROS 2 Trajectory mode and flies straight on its course at the held altitude until you press Finish.'
    : 'The UAV is in Trajectory mode and brakes to a hover ("stop and wait") until you press Finish.';
  updatePlannerAvailability();
  updatePlanStats();
  idInput.focus();
  $('view-hint').textContent = 'Click to add points · drag handles to shape · right-click to delete · drag map to pan';
}

function stopPlanning() {
  planner.active = false;
  planner.plan = null;
  views['2d'].setPlan(null);
  planEditor.hidden = true;
  updatePlannerAvailability();
  $('view-hint').textContent = 'Drag to pan · scroll to zoom';
}

function updatePlanStats() {
  const plan = planner.plan;
  if (!plan || plan.version === planner.renderedVersion) return;
  planner.renderedVersion = plan.version;
  const r = plan.sample();
  $('p-anchors').textContent = plan.length;
  $('p-points').textContent = r.points.length;
  $('p-length').textContent = r.points.length ? `${fmt(r.length, 1)} m` : '—';
  $('p-duration').textContent = r.points.length ? `${fmt(r.duration, 2)} s` : '—';
  $('p-speed').textContent = `${DEFAULT_SPEED} m/s`;
  $('p-at').textContent = r.points.length ? `${fmt(r.maxAt, 2)} m/s² (${fmt(r.maxAt / 9.81, 2)} g)` : '—';
  $('p-jt').textContent = r.points.length ? `${fmt(r.maxJt, 1)} m/s³` : '—';
  $('plan-undo').disabled = plan.length === 0;
  $('plan-clear').disabled = plan.length === 0;
}

function parseId() {
  const raw = idInput.value.trim();
  if (!/^\d+$/.test(raw)) return null;
  const id = Number(raw);
  return id <= 255 ? id : null;
}

// Planning starts by putting the UAV in Trajectory mode: PX4's trajectory manager only exists (and
// only receives the upload) while that mode is active.
planBtn.addEventListener('click', async () => {
  if (activeName !== '2d') return;
  const vehicle = selectedVehicle();
  if (vehicle?.heartbeat && inTrajectoryMode(vehicle)) {
    modeStatus.textContent = 'Already in Trajectory mode.';
    startPlanning();
    return;
  }
  planBtn.disabled = true;
  const accepted = await requestTrajectoryMode();
  planBtn.disabled = false;
  if (accepted) startPlanning();
});
$('plan-cancel').addEventListener('click', stopPlanning);
$('plan-undo').addEventListener('click', () => planner.plan?.undo());
$('plan-clear').addEventListener('click', () => planner.plan?.clear());
$('view-2d').addEventListener('plan-changed', () => (planError.textContent = ''));
// Enter in the ID field must not submit: uploading is only done with the Finish button
idInput.addEventListener('keydown', (ev) => {
  if (ev.key === 'Enter') ev.preventDefault();
});
idInput.addEventListener('input', () => {
  idInput.removeAttribute('aria-invalid');
  planError.textContent = '';
});
document.addEventListener('keydown', (ev) => {
  if (!planner.active || ev.target instanceof HTMLInputElement) return;
  if ((ev.ctrlKey || ev.metaKey) && ev.key.toLowerCase() === 'z') {
    ev.preventDefault();
    planner.plan.undo();
  }
});

planEditor.addEventListener('submit', (ev) => {
  ev.preventDefault();
  const fail = (msg) => (planError.textContent = msg);

  const id = parseId();
  if (id === null) {
    idInput.setAttribute('aria-invalid', 'true');
    idInput.focus();
    return fail('ID must be a whole number from 0 to 255.');
  }
  if (planner.plan.length < 2) return fail('Add at least two points on the map.');

  const { points } = planner.plan.sample();
  if (points.length > 65535) return fail(`Too many points (${points.length}); the index is a uint16.`);
  if (!telemetry.brokerConnected) return fail('Not connected to the MQTT broker.');
  if (!telemetry.bridgeOnline) return fail('The MAVLink bridge is offline.');
  const vehicle = selectedVehicle();
  if (!vehicle) return fail('No vehicle connected.');
  if (vehicle.heartbeat && !inTrajectoryMode(vehicle)) {
    // PX4's trajectory manager only exists in Trajectory mode; an upload now would be lost
    return fail('The UAV is not in Trajectory mode. Press Trajectory Flight Mode, then Finish again.');
  }

  rosConfirmedId = null;
  if (!telemetry.sendTrajectory(vehicle.sysid, id, points)) return fail('Could not publish the trajectory.');

  views['2d'].setUploadedPath(points);
  uploadStatus.textContent = isVtol(vehicle)
    ? `Trajectory ${id}: sending ${points.length} points to the ROS 2 mode…`
    : `Trajectory ${id}: sending ${points.length} points to system ${vehicle.sysid}…`;
  stopPlanning();
});

let rosConfirmedId = null; // last trajectory id the ROS 2 mode confirmed
telemetry.addEventListener('trajectory-status', (ev) => {
  const { sysid, id, state, sent, total, error } = ev.detail;
  // For a VTOL, PX4 forwards the same upload to the ROS 2 mode, which confirms it (below)
  const vtol = isVtol(telemetry.vehicles.get(sysid));
  if (vtol && state === 'done' && rosConfirmedId === id) return;
  if (state === 'error') {
    uploadStatus.textContent = `Trajectory ${id} rejected by the bridge: ${error}`;
  } else if (state === 'done' && vtol) {
    uploadStatus.textContent = `Trajectory ${id}: ${total} points sent, waiting for the ROS 2 mode to confirm…`;
  } else if (state === 'done') {
    uploadStatus.textContent =
      `Trajectory ${id} uploaded to system ${sysid}: INITIATE + ${total} UPLOAD messages ` +
      `(${SAMPLE_SPACING} m spacing, ${DEFAULT_SPEED} m/s).`;
  } else {
    uploadStatus.textContent = `Trajectory ${id}: uploading ${sent}/${total} points…`;
  }
});

telemetry.addEventListener('ros-trajectory-status', (ev) => {
  const { id, points, state } = ev.detail;
  rosConfirmedId = id;
  uploadStatus.textContent =
    state === 'loaded'
      ? `Trajectory ${id} loaded by the ROS 2 mode (${points} points, ${SAMPLE_SPACING} m spacing, ${DEFAULT_SPEED} m/s).`
      : `ROS 2 mode: trajectory ${id} ${state}.`;
});

updatePlannerAvailability();

if (storageGet(VIEW_KEY) === '3d') setView('3d');

// ---------------------------------------------------------------- render loop

let lastPanelUpdate = 0;

function frame(now) {
  const vehicles = [...telemetry.vehicles.values()];
  const selected = selectedVehicle();
  activeView.render(vehicles, selected);

  updatePlanStats();
  if (now - lastPanelUpdate > 100) {
    updatePanel(selected);
    updateGotoTarget(selected);
    lastPanelUpdate = now;
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

function updatePanel(v) {
  const now = performance.now();
  const fresh = v && now - Math.max(v.lastPositionAt, v.lastHeartbeatAt) < STALE_MS;
  setPill('st-vehicle', !v ? 'off' : fresh ? 'ok' : 'warn');

  const p = v?.position;
  const a = v?.attitude;
  const hb = v?.heartbeat;

  $('t-mode').textContent = hb ? modeName(v) : '—';
  const vtol = isVtol(v);
  for (const el of document.querySelectorAll('.vtol-only')) el.hidden = !vtol;
  if (v) updateParamSource(v);
  $('t-vtol').textContent = vtol ? vtolStateName(v) : '—';
  if (vtol && !takeoffAltTouched && takeoffAlt.value === '2.5') takeoffAlt.value = String(VTOL_TAKEOFF_ALT);
  $('t-armed').textContent = hb ? (hb.armed ? 'Armed' : 'Disarmed') : '—';
  $('t-rate').textContent = v ? `${v.positionRateHz.toFixed(0)} Hz` : '—';

  // ROS 2 fixed-wing mode guidance (published at 5 Hz while the mode runs)
  const g = v && now - v.rosGuidanceAt < 2500 ? v.rosGuidance : null;
  // Acquiring: r is the distance to the point being flown to (the path start, or where it was lost)
  const tracking = g?.state === 'tracking' || g?.state === 'acquiring';
  $('g-r').textContent = tracking && Number.isFinite(g.r) ? `${g.r.toFixed(1)} m` : '—';
  $('g-am').textContent = tracking ? fmt(g.a_m) : '—';
  $('g-along').textContent = tracking ? fmt(g.a_long) : '—';
  $('g-state').textContent = g
    ? {
        tracking: 'Tracking',
        acquiring: 'Acquiring the path',
        holding: 'Holding course (no target)',
        finished: 'Finished, holding course',
        inactive: 'Mode inactive',
      }[g.state] ?? g.state
    : 'Mode inactive';
  $('g-index').textContent = tracking && g.points ? `${g.index} / ${g.points - 1}` : '—';

  $('t-x').textContent = fmt(p?.x);
  $('t-y').textContent = fmt(p?.y);
  $('t-z').textContent = fmt(p?.z);
  $('t-alt').textContent = p ? `${fmt(-p.z)} m` : '—';
  $('t-dist').textContent = p ? `${fmt(Math.hypot(p.x, p.y))} m` : '—';

  $('t-vx').textContent = fmt(p?.vx);
  $('t-vy').textContent = fmt(p?.vy);
  $('t-vz').textContent = fmt(p?.vz);
  const gs = p ? Math.hypot(p.vx, p.vy) : NaN;
  $('t-gs').textContent = p ? `${fmt(gs)} m/s` : '—';
  $('t-course').textContent =
    p && gs > 0.2 ? `${fmt((deg(Math.atan2(p.vy, p.vx)) + 360) % 360, 0)}°` : '—';

  $('t-roll').textContent = fmt(a && deg(a.roll), 1);
  $('t-pitch').textContent = fmt(a && deg(a.pitch), 1);
  $('t-yaw').textContent = fmt(a && (deg(a.yaw) + 360) % 360, 1);
}
