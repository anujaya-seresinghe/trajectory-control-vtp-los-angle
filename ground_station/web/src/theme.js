const NAMES = [
  'scene-bg',
  'scene-grid',
  'scene-grid-major',
  'scene-axis-n',
  'scene-axis-e',
  'scene-label',
  'scene-trail',
  'scene-uav',
  'scene-uav-nose',
  'scene-velocity',
  'scene-plan',
  'scene-plan-handle',
  'scene-plan-handle-set',
];

/** Scene colours from the CSS custom properties in style.css, keyed without the "scene-" prefix. */
export function readSceneColors() {
  const style = getComputedStyle(document.documentElement);
  const colors = {};
  for (const name of NAMES) {
    colors[name.replace('scene-', '')] = style.getPropertyValue(`--${name}`).trim();
  }
  return colors;
}

export function onThemeChange(callback) {
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', callback);
}
