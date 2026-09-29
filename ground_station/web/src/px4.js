// Decoding of PX4's HEARTBEAT.custom_mode (see PX4/src/modules/commander/px4_custom_mode.h)

// PX4 custom mode for the trajectory-following flight mode (PX4_CUSTOM_SUB_MODE_AUTO_TRAJ)
export const PX4_MAIN_MODE_AUTO = 4;
export const PX4_SUB_MODE_AUTO_TRAJ = 9;
export const PX4_SUB_MODE_AUTO_LOITER = 3; // "Hold"

// Name the ROS 2 fixed-wing mode (ros2/src/trajectory_mode_fw) registers with
export const ROS_TRAJECTORY_MODE_NAME = 'Trajectory';

const MAV_TYPES_VTOL = new Set([19, 20, 21, 22, 23, 24, 47]);
export const MAV_VTOL_STATE = { UNDEFINED: 0, TRANSITION_TO_FW: 1, TRANSITION_TO_MC: 2, MC: 3, FW: 4 };
const VTOL_STATE_NAMES = ['—', 'Transition to FW', 'Transition to MC', 'Multicopter', 'Fixed-wing'];

export function isVtol(vehicle) {
  if (!vehicle) return false;
  if (MAV_TYPES_VTOL.has(vehicle.heartbeat?.type)) return true;
  return Boolean(vehicle.extState && vehicle.extState.vtol_state !== MAV_VTOL_STATE.UNDEFINED);
}

export function vtolStateName(vehicle) {
  return VTOL_STATE_NAMES[vehicle?.extState?.vtol_state ?? 0] ?? '—';
}

/** PX4 external (ROS 2) modes: AUTO / EXTERNAL1..8 */
export function isExternalMode(customMode) {
  const sub = (customMode >>> 24) & 0xff;
  return ((customMode >>> 16) & 0xff) === PX4_MAIN_MODE_AUTO && sub >= 12 && sub <= 19;
}

/** custom_mode of the registered ROS 2 "Trajectory" mode, or null if it is not (yet) registered */
export function rosTrajectoryMode(vehicle) {
  for (const [customMode, mode] of vehicle?.modes ?? []) {
    if (mode.name === ROS_TRAJECTORY_MODE_NAME && isExternalMode(customMode)) return customMode;
  }
  return null;
}

export function isTrajectoryMode(customMode) {
  return ((customMode >>> 16) & 0xff) === PX4_MAIN_MODE_AUTO && ((customMode >>> 24) & 0xff) === PX4_SUB_MODE_AUTO_TRAJ;
}

// Trajectory controller parameters (PX4/src/modules/flight_mode_manager/tasks/Traj/flight_task_traj_params.yaml)
export const TRAJ_PARAMS = [
  { name: 'TRAJ_R_STAR', label: 'r*', unit: 'm', min: 1, max: 200, step: 0.5 },
  { name: 'TRAJ_SMC_ALPHA', label: 'α', min: 1.01, max: 1.99, step: 0.01 },
  { name: 'TRAJ_SMC_BETA', label: 'β', min: 0.001, max: 10, step: 0.01 },
  { name: 'TRAJ_SMC_EPS', label: 'ε', min: 0, max: 10, step: 0.01 },
  { name: 'TRAJ_K_LONG', label: 'k_long', min: 0, max: 10, step: 0.05 },
  { name: 'TRAJ_A_M_MAX', label: '|a_m|', unit: 'm/s²', min: 0, max: 20, step: 0.1 },
  { name: 'TRAJ_A_LONG_MAX', label: '|a_long|', unit: 'm/s²', min: 0, max: 10, step: 0.1 },
];

const MAIN_MODES = {
  1: 'Manual',
  2: 'Altitude',
  3: 'Position',
  4: 'Auto',
  5: 'Acro',
  6: 'Offboard',
  7: 'Stabilized',
  8: 'Rattitude',
  10: 'Termination',
  11: 'Altitude Cruise',
};

const AUTO_SUB_MODES = {
  1: 'Ready',
  2: 'Takeoff',
  3: 'Hold',
  4: 'Mission',
  5: 'Return',
  6: 'Land',
  8: 'Follow Target',
  9: 'Trajectory',
  10: 'Precision Land',
  11: 'VTOL Takeoff',
  12: 'External 1',
  13: 'External 2',
  14: 'External 3',
  15: 'External 4',
  16: 'External 5',
  17: 'External 6',
  18: 'External 7',
  19: 'External 8',
  20: 'Guided Course',
  21: 'Descend',
};

const POSCTL_SUB_MODES = { 1: 'Orbit', 2: 'Slow' };

export function px4ModeName(customMode) {
  const main = (customMode >>> 16) & 0xff;
  const sub = (customMode >>> 24) & 0xff;

  if (main === 4) return AUTO_SUB_MODES[sub] ?? `Auto (${sub})`;
  if (main === 3 && POSCTL_SUB_MODES[sub]) return POSCTL_SUB_MODES[sub];
  return MAIN_MODES[main] ?? `Unknown (${main}/${sub})`;
}

// MAV_RESULT values of COMMAND_ACK
const MAV_RESULTS = {
  0: 'accepted',
  1: 'temporarily rejected',
  2: 'denied',
  3: 'unsupported',
  4: 'failed',
  5: 'in progress',
  6: 'cancelled',
};

export function mavResultName(result) {
  return MAV_RESULTS[result] ?? `result ${result}`;
}
