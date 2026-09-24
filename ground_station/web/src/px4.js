// Decoding of PX4's HEARTBEAT.custom_mode (see PX4/src/modules/commander/px4_custom_mode.h)

// PX4 custom mode for the trajectory-following flight mode (PX4_CUSTOM_SUB_MODE_AUTO_TRAJ)
export const PX4_MAIN_MODE_AUTO = 4;
export const PX4_SUB_MODE_AUTO_TRAJ = 9;

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
