import { Telemetry } from './telemetry.js';
import { px4ModeName, mavResultName, PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_TRAJ } from './px4.js';
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

// ---------------------------------------------------------------- trajectory flight mode

const MAV_CMD_DO_SET_MODE = 176;
const ACK_TIMEOUT_MS = 3000;
const modeStatus = $('mode-status');
let modeAckTimer = null;

$('traj-mode-btn').addEventListener('click', () => {
  const vehicle = selectedVehicle();
  if (!telemetry.brokerConnected) return (modeStatus.textContent = 'Not connected to the MQTT broker.');
  if (!telemetry.bridgeOnline) return (modeStatus.textContent = 'The MAVLink bridge is offline.');
  if (!vehicle) return (modeStatus.textContent = 'No vehicle connected.');

  telemetry.sendSetMode(vehicle.sysid, PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_TRAJ);
  modeStatus.textContent = `Requesting Trajectory mode on system ${vehicle.sysid}…`;
  clearTimeout(modeAckTimer);
  modeAckTimer = setTimeout(() => (modeStatus.textContent = 'No ACK received from the vehicle.'), ACK_TIMEOUT_MS);
});

telemetry.addEventListener('command-ack', (ev) => {
  const { sysid, command, result } = ev.detail;
  if (command !== MAV_CMD_DO_SET_MODE || modeAckTimer === null) return;
  clearTimeout(modeAckTimer);
  modeAckTimer = null;
  modeStatus.textContent =
    result === 0
      ? `Trajectory mode accepted by system ${sysid}.`
      : `Trajectory mode ${mavResultName(result)} by system ${sysid}.`;
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

planBtn.addEventListener('click', startPlanning);
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

  if (!telemetry.sendTrajectory(vehicle.sysid, id, points)) return fail('Could not publish the trajectory.');

  views['2d'].setUploadedPath(points);
  uploadStatus.textContent = `Trajectory ${id}: sending ${points.length} points to system ${vehicle.sysid}…`;
  stopPlanning();
});

telemetry.addEventListener('trajectory-status', (ev) => {
  const { sysid, id, state, sent, total, error } = ev.detail;
  if (state === 'error') {
    uploadStatus.textContent = `Trajectory ${id} rejected by the bridge: ${error}`;
  } else if (state === 'done') {
    uploadStatus.textContent =
      `Trajectory ${id} uploaded to system ${sysid}: INITIATE + ${total} UPLOAD messages ` +
      `(${SAMPLE_SPACING} m spacing, ${DEFAULT_SPEED} m/s).`;
  } else {
    uploadStatus.textContent = `Trajectory ${id}: uploading ${sent}/${total} points…`;
  }
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

  $('t-mode').textContent = hb ? px4ModeName(hb.custom_mode) : '—';
  $('t-armed').textContent = hb ? (hb.armed ? 'Armed' : 'Disarmed') : '—';
  $('t-rate').textContent = v ? `${v.positionRateHz.toFixed(0)} Hz` : '—';

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
