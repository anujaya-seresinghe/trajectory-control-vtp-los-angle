import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { readSceneColors, onThemeChange } from './theme.js';

const TRAIL_CAPACITY = 20000;
const MODEL_SCALE = 4; // exaggerate the airframe so it stays visible at trajectory scale

/**
 * NED -> three.js (Y up, right-handed): X = East, Y = Up, Z = South.
 */
export function nedToThree(n, e, d, out = new THREE.Vector3()) {
  return out.set(e, -d, -n);
}

/**
 * Body attitude (NED ZYX Euler: yaw, pitch, roll) -> three.js Euler.
 * The model's forward axis is -Z, right is +X and down is -Y, i.e. the same mapping as nedToThree,
 * so R_three = M * R_ned * M^T which works out to Ry(-yaw) * Rx(pitch) * Rz(-roll).
 */
function attitudeToEuler(roll, pitch, yaw, out = new THREE.Euler()) {
  return out.set(pitch, -yaw, -roll, 'YXZ');
}

export class View3D {
  constructor(host) {
    this.host = host;
    this.follow = true;
    this.colors = readSceneColors();
    this.entries = new Map(); // sysid -> scene objects

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    host.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.1, 5000);
    this.camera.position.set(-14, 14, 18);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.target.set(0, 5, 0);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(30, 60, 20);
    this.scene.add(sun);

    this._buildEnvironment();
    this._applyTheme();
    onThemeChange(() => {
      this.colors = readSceneColors();
      this._applyTheme();
    });

    this._lastFollowPos = null;
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
  }

  setFollow(follow) {
    this.follow = follow;
    this._lastFollowPos = null;
  }

  recenter(vehicle) {
    const p = vehicle?.position;
    const target = p ? nedToThree(p.x, p.y, p.z) : new THREE.Vector3(0, 5, 0);
    const offset = new THREE.Vector3(-14, 10, 18);
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(offset);
    this._lastFollowPos = null;
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.host;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render(vehicles, selected) {
    for (const v of vehicles) this._updateVehicle(v, v === selected);

    if (this.follow && selected?.position) {
      const pos = nedToThree(selected.position.x, selected.position.y, selected.position.z);
      if (this._lastFollowPos) {
        // Translate camera and target together so the user's orbit angle/distance is preserved
        const delta = pos.clone().sub(this._lastFollowPos);
        this.camera.position.add(delta);
        this.controls.target.add(delta);
      } else {
        const offset = this.camera.position.clone().sub(this.controls.target);
        this.controls.target.copy(pos);
        this.camera.position.copy(pos).add(offset);
      }
      this._lastFollowPos = pos;
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  // ---------------------------------------------------------------- scene setup

  _buildEnvironment() {
    this.gridMinor = new THREE.GridHelper(400, 400);
    this.gridMajor = new THREE.GridHelper(400, 40);
    this.gridMinor.material.transparent = true;
    this.gridMinor.material.opacity = 0.5;
    this.gridMajor.position.y = 0.01;
    this.scene.add(this.gridMinor, this.gridMajor);

    // NED origin axes on the ground: North (-Z) and East (+X)
    const axisLen = 10;
    this.axisN = makeLine([new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(0, 0.02, -axisLen)]);
    this.axisE = makeLine([new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(axisLen, 0.02, 0)]);
    this.scene.add(this.axisN, this.axisE);

    this.labelN = makeTextSprite('N');
    this.labelN.position.set(0, 0.8, -axisLen - 1);
    this.labelE = makeTextSprite('E');
    this.labelE.position.set(axisLen + 1, 0.8, 0);
    this.scene.add(this.labelN, this.labelE);
  }

  _applyTheme() {
    const c = this.colors;
    this.scene.background = new THREE.Color(c.bg);
    this.gridMinor.material.color.set(c.grid);
    this.gridMajor.material.color.set(c['grid-major']);
    this.axisN.material.color.set(c['axis-n']);
    this.axisE.material.color.set(c['axis-e']);
    setSpriteColor(this.labelN, c['axis-n']);
    setSpriteColor(this.labelE, c['axis-e']);

    for (const entry of this.entries.values()) {
      entry.trail.material.color.set(c.trail);
      entry.shadowTrail.material.color.set(c.trail);
      entry.stalk.material.color.set(c.label);
      entry.bodyMaterial.color.set(c.uav);
      entry.noseMaterial.color.set(c['uav-nose']);
      entry.velocity.material.color.set(c.velocity);
    }
  }

  _entry(vehicle) {
    let entry = this.entries.get(vehicle.sysid);
    if (entry) return entry;

    const c = this.colors;

    const trailGeom = new THREE.BufferGeometry();
    trailGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_CAPACITY * 3), 3));
    trailGeom.setDrawRange(0, 0);
    const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: c.trail }));
    trail.frustumCulled = false;

    // Ground projection of the trail to make the path readable in 3D
    const shadowGeom = new THREE.BufferGeometry();
    shadowGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_CAPACITY * 3), 3));
    shadowGeom.setDrawRange(0, 0);
    const shadowTrail = new THREE.Line(
      shadowGeom,
      new THREE.LineBasicMaterial({ color: c.trail, transparent: true, opacity: 0.25 }),
    );
    shadowTrail.frustumCulled = false;
    shadowTrail.position.y = 0.03;

    const stalk = makeLine([new THREE.Vector3(), new THREE.Vector3()], { dashed: true });
    stalk.material.color.set(c.label);
    const velocity = makeLine([new THREE.Vector3(), new THREE.Vector3()]);
    velocity.material.color.set(c.velocity);

    const bodyMaterial = new THREE.MeshStandardMaterial({ color: c.uav, roughness: 0.6, metalness: 0.2 });
    const noseMaterial = new THREE.MeshStandardMaterial({ color: c['uav-nose'], roughness: 0.6 });
    const model = buildQuadModel(bodyMaterial, noseMaterial);
    model.scale.setScalar(MODEL_SCALE);

    this.scene.add(trail, shadowTrail, stalk, velocity, model);
    entry = { trail, shadowTrail, stalk, velocity, model, bodyMaterial, noseMaterial, trailVersion: -1 };
    this.entries.set(vehicle.sysid, entry);
    return entry;
  }

  _updateVehicle(vehicle, isSelected) {
    if (!vehicle.position) return;
    const entry = this._entry(vehicle);
    const p = vehicle.position;
    const pos = nedToThree(p.x, p.y, p.z);

    entry.model.position.copy(pos);
    const att = vehicle.attitude;
    attitudeToEuler(att?.roll ?? 0, att?.pitch ?? 0, att?.yaw ?? Math.atan2(p.vy, p.vx), entry.model.rotation);

    const stalkPos = entry.stalk.geometry.attributes.position;
    stalkPos.setXYZ(0, pos.x, pos.y, pos.z);
    stalkPos.setXYZ(1, pos.x, 0, pos.z);
    stalkPos.needsUpdate = true;
    entry.stalk.computeLineDistances();

    const velPos = entry.velocity.geometry.attributes.position;
    const tip = nedToThree(p.x + p.vx, p.y + p.vy, p.z + p.vz);
    velPos.setXYZ(0, pos.x, pos.y, pos.z);
    velPos.setXYZ(1, tip.x, tip.y, tip.z);
    velPos.needsUpdate = true;

    entry.trail.material.opacity = isSelected ? 1 : 0.4;
    entry.trail.material.transparent = !isSelected;

    if (entry.trailVersion !== vehicle.trailVersion) {
      writeTrail(entry.trail.geometry, vehicle.trail, false);
      writeTrail(entry.shadowTrail.geometry, vehicle.trail, true);
      entry.trailVersion = vehicle.trailVersion;
    }
  }
}

// ---------------------------------------------------------------- helpers

function writeTrail(geometry, trail, flatten) {
  const attr = geometry.attributes.position;
  const count = Math.min(trail.length, TRAIL_CAPACITY);
  const start = trail.length - count;
  const v = new THREE.Vector3();
  for (let i = 0; i < count; i++) {
    const [n, e, d] = trail[start + i];
    nedToThree(n, e, flatten ? 0 : d, v);
    attr.setXYZ(i, v.x, v.y, v.z);
  }
  attr.needsUpdate = true;
  geometry.setDrawRange(0, count);
}

function makeLine(points, { dashed = false } = {}) {
  const geom = new THREE.BufferGeometry().setFromPoints(points);
  const material = dashed
    ? new THREE.LineDashedMaterial({ dashSize: 0.5, gapSize: 0.4, transparent: true, opacity: 0.7 })
    : new THREE.LineBasicMaterial();
  const line = new THREE.Line(geom, material);
  line.frustumCulled = false;
  if (dashed) line.computeLineDistances();
  return line;
}

/** X-configuration quadrotor, ~0.5 m motor-to-motor. Forward is -Z. */
function buildQuadModel(bodyMaterial, noseMaterial) {
  const group = new THREE.Group();

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.07, 0.24), bodyMaterial);
  group.add(body);

  const armGeom = new THREE.BoxGeometry(0.025, 0.02, 0.36);
  const rotorGeom = new THREE.CylinderGeometry(0.11, 0.11, 0.008, 24);
  const arms = [
    { angle: Math.PI / 4, front: true },
    { angle: -Math.PI / 4, front: true },
    { angle: (3 * Math.PI) / 4, front: false },
    { angle: (-3 * Math.PI) / 4, front: false },
  ];
  for (const { angle, front } of arms) {
    const material = front ? noseMaterial : bodyMaterial;
    const dir = new THREE.Vector3(Math.sin(angle), 0, -Math.cos(angle));
    const arm = new THREE.Mesh(armGeom, material);
    arm.position.copy(dir.clone().multiplyScalar(0.18));
    arm.rotation.y = -angle;
    group.add(arm);

    const rotor = new THREE.Mesh(
      rotorGeom,
      new THREE.MeshStandardMaterial({ color: material.color, transparent: true, opacity: 0.35 }),
    );
    rotor.material.color = material.color; // share colour so theme changes propagate
    rotor.position.copy(dir.clone().multiplyScalar(0.36)).setY(0.04);
    group.add(rotor);
  }

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.035, 0.1, 12), noseMaterial);
  nose.rotation.x = -Math.PI / 2;
  nose.position.set(0, 0, -0.17);
  group.add(nose);

  return group;
}

function makeTextSprite(text) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 64;
  const texture = new THREE.CanvasTexture(canvas);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }));
  sprite.scale.set(2, 2, 1);
  sprite.userData = { canvas, text };
  return sprite;
}

function setSpriteColor(sprite, color) {
  const { canvas, text } = sprite.userData;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = color;
  ctx.font = '600 44px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 32, 34);
  sprite.material.map.needsUpdate = true;
}
