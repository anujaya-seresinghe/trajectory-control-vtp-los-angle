import { readSceneColors, onThemeChange } from './theme.js';
import { SAMPLE_SPACING } from './trajectory.js';
import { isVtol } from './px4.js';

const HIT_RADIUS = 10; // [px] pick radius for anchors and handles

/**
 * Top-down view of the local NED frame: North is up, East is right.
 * World (x=N, y=E) -> screen: sx = cx + (y - centerE) * scale, sy = cy - (x - centerN) * scale
 */
export class View2D {
  constructor(host) {
    this.host = host;
    this.canvas = document.createElement('canvas');
    host.appendChild(this.canvas);
    this.ctx = this.canvas.getContext('2d');

    this.center = { n: 0, e: 0 };
    this.scale = 8; // pixels per metre
    this.follow = true;
    this.plan = null; // TrajectoryPlan being edited, or null
    this.uploadedPath = null; // [{x, y}, ...] of the last uploaded trajectory
    this.gotoTarget = null; // {n, e} of an active "Go here"
    this.colors = readSceneColors();
    onThemeChange(() => (this.colors = readSceneColors()));

    this._bindInput();
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
  }

  setFollow(follow) {
    this.follow = follow;
  }

  /** Enable trajectory editing on the map (pass null to leave planning mode). */
  setPlan(plan) {
    this.plan = plan;
    this.canvas.style.cursor = plan ? 'crosshair' : '';
  }

  setGotoTarget(target) {
    this.gotoTarget = target;
  }

  setUploadedPath(points) {
    this.uploadedPath = points;
  }

  recenter(vehicle) {
    const p = vehicle?.position;
    this.center = p ? { n: p.x, e: p.y } : { n: 0, e: 0 };
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const { clientWidth: w, clientHeight: h } = this.host;
    this.canvas.width = Math.max(1, Math.round(w * dpr));
    this.canvas.height = Math.max(1, Math.round(h * dpr));
    this.width = w;
    this.height = h;
    this.dpr = dpr;
  }

  toScreen(n, e) {
    return [this.width / 2 + (e - this.center.e) * this.scale, this.height / 2 - (n - this.center.n) * this.scale];
  }

  toWorld(sx, sy) {
    return {
      n: this.center.n - (sy - this.height / 2) / this.scale,
      e: this.center.e + (sx - this.width / 2) / this.scale,
    };
  }

  render(vehicles, selected) {
    const { ctx, colors } = this;

    if (this.follow && selected?.position) {
      this.center = { n: selected.position.x, e: selected.position.y };
    }

    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.fillStyle = colors.bg;
    ctx.fillRect(0, 0, this.width, this.height);

    this._drawGrid();
    this._drawOrigin();
    this._drawUploadedPath();
    this._drawGotoTarget();

    for (const v of vehicles) {
      this._drawTrail(v, v === selected);
    }
    for (const v of vehicles) {
      this._drawVehicle(v, v === selected);
    }

    if (this.plan) this._drawPlan(this.plan);

    this._drawCompass();
    this._drawScaleBar();
  }

  // ---------------------------------------------------------------- drawing

  _gridStep() {
    // 1-2-5 sequence so grid lines are roughly 60-150 px apart
    const target = 80 / this.scale;
    const pow = Math.pow(10, Math.floor(Math.log10(target)));
    for (const m of [1, 2, 5, 10]) {
      if (m * pow >= target) return m * pow;
    }
    return 10 * pow;
  }

  _drawGrid() {
    const { ctx, colors } = this;
    const step = this._gridStep();
    const topLeft = this.toWorld(0, 0);
    const bottomRight = this.toWorld(this.width, this.height);

    ctx.lineWidth = 1;
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillStyle = colors.label;

    // Lines of constant East (vertical)
    for (let e = Math.ceil(topLeft.e / step) * step; e <= bottomRight.e; e += step) {
      const [x] = this.toScreen(0, e);
      const major = Math.abs(e / (step * 5) - Math.round(e / (step * 5))) < 1e-6;
      ctx.strokeStyle = major ? colors['grid-major'] : colors.grid;
      ctx.beginPath();
      ctx.moveTo(Math.round(x) + 0.5, 0);
      ctx.lineTo(Math.round(x) + 0.5, this.height);
      ctx.stroke();
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText(formatMetres(e), x + 3, 3);
    }

    // Lines of constant North (horizontal)
    for (let n = Math.ceil(bottomRight.n / step) * step; n <= topLeft.n; n += step) {
      const [, y] = this.toScreen(n, 0);
      const major = Math.abs(n / (step * 5) - Math.round(n / (step * 5))) < 1e-6;
      ctx.strokeStyle = major ? colors['grid-major'] : colors.grid;
      ctx.beginPath();
      ctx.moveTo(0, Math.round(y) + 0.5);
      ctx.lineTo(this.width, Math.round(y) + 0.5);
      ctx.stroke();
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      ctx.fillText(formatMetres(n), 3, y - 2);
    }
  }

  _drawOrigin() {
    const { ctx, colors } = this;
    const [x, y] = this.toScreen(0, 0);
    const len = 36;

    ctx.lineWidth = 2;
    ctx.strokeStyle = colors['axis-n'];
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x, y - len);
    ctx.stroke();
    ctx.strokeStyle = colors['axis-e'];
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + len, y);
    ctx.stroke();

    ctx.font = '600 11px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillStyle = colors['axis-n'];
    ctx.fillText('N', x, y - len - 2);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = colors['axis-e'];
    ctx.fillText('E', x + len + 3, y);

    ctx.fillStyle = colors.label;
    ctx.beginPath();
    ctx.arc(x, y, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  _drawTrail(vehicle, isSelected) {
    const { ctx, colors } = this;
    const trail = vehicle.trail;
    if (trail.length < 2) return;

    ctx.strokeStyle = colors.trail;
    ctx.globalAlpha = isSelected ? 0.9 : 0.4;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    let [x, y] = this.toScreen(trail[0][0], trail[0][1]);
    ctx.moveTo(x, y);
    for (let i = 1; i < trail.length; i++) {
      [x, y] = this.toScreen(trail[i][0], trail[i][1]);
      ctx.lineTo(x, y);
    }
    if (vehicle.position) {
      [x, y] = this.toScreen(vehicle.position.x, vehicle.position.y);
      ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  _drawVehicle(vehicle, isSelected) {
    const { ctx, colors } = this;
    const p = vehicle.position;
    if (!p) return;

    const [x, y] = this.toScreen(p.x, p.y);
    const yaw = vehicle.attitude?.yaw ?? Math.atan2(p.vy, p.vx);

    // Velocity vector (1 s look-ahead)
    const [vx, vy] = this.toScreen(p.x + p.vx, p.y + p.vy);
    ctx.strokeStyle = colors.velocity;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(vx, vy);
    ctx.stroke();

    // Body along the heading. Canvas rotation is clockwise, same as NED yaw.
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(yaw);
    const s = isSelected ? 1 : 0.8;
    if (isVtol(vehicle)) drawVtolIcon(ctx, colors, s);
    else drawArrowIcon(ctx, colors, s);
    ctx.restore();

    // Label with system id and altitude
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    const label = `#${vehicle.sysid}  ${(-p.z).toFixed(1)} m`;
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = colors.bg;
    ctx.globalAlpha = 0.85;
    ctx.fillRect(x + 18, y - 10, tw + 10, 20);
    ctx.globalAlpha = 1;
    ctx.fillStyle = colors.uav;
    ctx.fillText(label, x + 23, y);
  }

  _drawGotoTarget() {
    if (!this.gotoTarget) return;
    const { ctx, colors } = this;
    const [x, y] = this.toScreen(this.gotoTarget.n, this.gotoTarget.e);
    ctx.strokeStyle = colors.velocity;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 9, 0, Math.PI * 2);
    ctx.moveTo(x - 14, y);
    ctx.lineTo(x - 4, y);
    ctx.moveTo(x + 4, y);
    ctx.lineTo(x + 14, y);
    ctx.moveTo(x, y - 14);
    ctx.lineTo(x, y - 4);
    ctx.moveTo(x, y + 4);
    ctx.lineTo(x, y + 14);
    ctx.stroke();
  }

  _drawUploadedPath() {
    const pts = this.uploadedPath;
    if (!pts || pts.length < 2) return;
    const { ctx, colors } = this;
    ctx.strokeStyle = colors.plan;
    ctx.globalAlpha = 0.45;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [x, y] = this.toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  _drawPlan(plan) {
    const { ctx, colors } = this;
    const anchors = plan.anchors;
    if (!anchors.length) return;

    // Curve: the Bézier segments map exactly onto canvas curves
    const segs = plan.segments();
    if (segs.length) {
      ctx.strokeStyle = colors.plan;
      ctx.lineWidth = 3;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      ctx.moveTo(...this.toScreen(...segs[0][0]));
      for (const [, c1, c2, p] of segs) {
        ctx.bezierCurveTo(...this.toScreen(...c1), ...this.toScreen(...c2), ...this.toScreen(...p));
      }
      ctx.stroke();

      // Uploaded sample points, once they are far enough apart on screen to tell apart
      if (SAMPLE_SPACING * this.scale >= 6) {
        ctx.fillStyle = colors.plan;
        for (const p of plan.sample().points) {
          const [x, y] = this.toScreen(p.x, p.y);
          if (x < -5 || y < -5 || x > this.width + 5 || y > this.height + 5) continue;
          ctx.beginPath();
          ctx.arc(x, y, 2, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    // Handles
    ctx.lineWidth = 1.5;
    anchors.forEach((a, i) => {
      const [ax, ay] = this.toScreen(a.n, a.e);
      const h = plan.handles(i);
      const color = plan.isClamped(i) ? colors['plan-handle-set'] : colors['plan-handle'];
      for (const hp of [h.in, h.out]) {
        if (!hp) continue;
        const [hx, hy] = this.toScreen(hp.n, hp.e);
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(hx, hy);
        ctx.stroke();
        ctx.fillStyle = colors.bg;
        ctx.beginPath();
        ctx.rect(hx - 4, hy - 4, 8, 8);
        ctx.fill();
        ctx.stroke();
      }
    });

    // Anchors (start drawn as a ring)
    anchors.forEach((a, i) => {
      const [x, y] = this.toScreen(a.n, a.e);
      ctx.beginPath();
      ctx.arc(x, y, 6, 0, Math.PI * 2);
      ctx.fillStyle = i === 0 ? colors.bg : colors.plan;
      ctx.strokeStyle = i === 0 ? colors.plan : colors.bg;
      ctx.lineWidth = i === 0 ? 3 : 2;
      ctx.fill();
      ctx.stroke();
    });

    const [sx, sy] = this.toScreen(anchors[0].n, anchors[0].e);
    ctx.font = '600 11px system-ui, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillStyle = colors.plan;
    ctx.fillText('Start', sx + 9, sy - 6);
  }

  /** Closest anchor or handle under screen point (sx, sy). */
  _hitTest(sx, sy) {
    const plan = this.plan;
    if (!plan) return null;
    let best = null;
    let bestDist = HIT_RADIUS;
    const consider = (target, n, e) => {
      const [x, y] = this.toScreen(n, e);
      const d = Math.hypot(x - sx, y - sy);
      if (d <= bestDist) {
        best = target;
        bestDist = d;
      }
    };
    plan.anchors.forEach((a, i) => {
      const h = plan.handles(i);
      const [ax, ay] = this.toScreen(a.n, a.e);
      for (const side of ['in', 'out']) {
        const hp = h[side];
        if (!hp) continue;
        const [hx, hy] = this.toScreen(hp.n, hp.e);
        // a handle lying on its anchor would make the anchor impossible to grab
        if (Math.hypot(hx - ax, hy - ay) > HIT_RADIUS * 1.5) consider({ type: 'handle', index: i, side }, hp.n, hp.e);
      }
    });
    // Anchors win ties with handles
    plan.anchors.forEach((a, i) => {
      const [x, y] = this.toScreen(a.n, a.e);
      if (Math.hypot(x - sx, y - sy) <= Math.max(bestDist, 8)) {
        best = { type: 'anchor', index: i };
        bestDist = Math.hypot(x - sx, y - sy);
      }
    });
    return best;
  }

  _drawCompass() {
    const { ctx, colors } = this;
    const x = this.width - 34;
    const y = 34;
    ctx.strokeStyle = colors['grid-major'];
    ctx.fillStyle = colors.bg;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(x, y, 20, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = colors['axis-n'];
    ctx.beginPath();
    ctx.moveTo(x, y - 15);
    ctx.lineTo(x + 5, y);
    ctx.lineTo(x - 5, y);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = colors.label;
    ctx.font = '600 10px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('N', x, y + 9);
  }

  _drawScaleBar() {
    const { ctx, colors } = this;
    const step = this._gridStep();
    const px = step * this.scale;
    const x = this.width - px - 16;
    const y = this.height - 18;
    ctx.strokeStyle = colors.label;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 5);
    ctx.lineTo(x, y);
    ctx.lineTo(x + px, y);
    ctx.lineTo(x + px, y - 5);
    ctx.stroke();
    ctx.fillStyle = colors.label;
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(formatMetres(step), x + px / 2, y - 3);
  }

  // ---------------------------------------------------------------- input

  _bindInput() {
    let drag = null;
    const local = (ev) => {
      const rect = this.canvas.getBoundingClientRect();
      return [ev.clientX - rect.left, ev.clientY - rect.top];
    };
    const planChanged = () => this.host.dispatchEvent(new CustomEvent('plan-changed', { bubbles: true }));

    this.canvas.addEventListener('pointerdown', (ev) => {
      this.host.dispatchEvent(new CustomEvent('map-pointerdown', { bubbles: true }));
      if (ev.button !== 0) return;
      const [sx, sy] = local(ev);
      const hit = this._hitTest(sx, sy);
      drag = { x: ev.clientX, y: ev.clientY, center: { ...this.center }, hit, moved: false };
      this.canvas.setPointerCapture(ev.pointerId);
    });

    this.canvas.addEventListener('pointermove', (ev) => {
      const [sx, sy] = local(ev);
      if (!drag) {
        if (this.plan) this.canvas.style.cursor = this._hitTest(sx, sy) ? 'move' : 'crosshair';
        return;
      }
      const dx = ev.clientX - drag.x;
      const dy = ev.clientY - drag.y;
      if (!drag.moved && Math.hypot(dx, dy) <= 3) return;
      drag.moved = true;

      const w = this.toWorld(sx, sy);
      if (drag.hit?.type === 'anchor') {
        this.plan.move(drag.hit.index, w.n, w.e);
        planChanged();
      } else if (drag.hit?.type === 'handle') {
        this.plan.setHandle(drag.hit.index, drag.hit.side, w.n, w.e);
        planChanged();
      } else {
        if (this.follow) this.host.dispatchEvent(new CustomEvent('user-pan', { bubbles: true }));
        this.center = { n: drag.center.n + dy / this.scale, e: drag.center.e - dx / this.scale };
      }
    });

    this.canvas.addEventListener('pointerup', (ev) => {
      if (drag && !drag.moved && !drag.hit && this.plan) {
        // A click (not a drag) on empty map appends a point
        const w = this.toWorld(...local(ev));
        this.plan.add(w.n, w.e);
        planChanged();
      }
      drag = null;
    });
    this.canvas.addEventListener('pointercancel', () => (drag = null));

    // Right-click: delete a point, or reset a handle to automatic
    this.canvas.addEventListener('contextmenu', (ev) => {
      ev.preventDefault();
      if (!this.plan) {
        // Outside planning, right-click opens the map context popup ("Go here")
        const [sx, sy] = local(ev);
        const w = this.toWorld(sx, sy);
        this.host.dispatchEvent(new CustomEvent('map-context', { bubbles: true, detail: { sx, sy, ...w } }));
        return;
      }
      const hit = this._hitTest(...local(ev));
      if (hit?.type === 'anchor') this.plan.remove(hit.index);
      else if (hit?.type === 'handle') this.plan.resetHandles(hit.index);
      else return;
      planChanged();
    });

    this.canvas.addEventListener(
      'wheel',
      (ev) => {
        ev.preventDefault();
        this.host.dispatchEvent(new CustomEvent('map-pointerdown', { bubbles: true }));
        const [sx, sy] = local(ev);
        const before = this.toWorld(sx, sy);
        const factor = Math.exp(-ev.deltaY * 0.0015);
        this.scale = Math.min(400, Math.max(0.05, this.scale * factor));
        if (!this.follow) {
          // Keep the point under the cursor fixed
          const after = this.toWorld(sx, sy);
          this.center.n += before.n - after.n;
          this.center.e += before.e - after.e;
        }
      },
      { passive: false },
    );
  }
}

/** Multicopter: arrow pointing along the heading (nose at -y). */
function drawArrowIcon(ctx, colors, s) {
  ctx.fillStyle = colors.uav;
  ctx.strokeStyle = colors.bg;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, -16 * s);
  ctx.lineTo(10 * s, 11 * s);
  ctx.lineTo(0, 6 * s);
  ctx.lineTo(-10 * s, 11 * s);
  ctx.closePath();
  ctx.stroke();
  ctx.fill();
  ctx.fillStyle = colors['uav-nose'];
  ctx.beginPath();
  ctx.arc(0, -10 * s, 2.5 * s, 0, Math.PI * 2);
  ctx.fill();
}

/** Standard VTOL (quadplane) seen from above: fuselage, wing, tail, two booms with four lift rotors (nose at -y). */
function drawVtolIcon(ctx, colors, s) {
  // Lift rotors, behind the airframe
  ctx.fillStyle = colors.uav;
  ctx.globalAlpha = 0.3;
  for (const [rx, ry] of [[-10, -10], [10, -10], [-10, 11], [10, 11]]) {
    ctx.beginPath();
    ctx.arc(rx * s, ry * s, 5 * s, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;

  ctx.fillStyle = colors.uav;
  ctx.strokeStyle = colors.bg;
  ctx.lineWidth = 1.5;
  const rect = (x0, y0, w, h) => {
    ctx.beginPath();
    ctx.rect(x0 * s, y0 * s, w * s, h * s);
    ctx.stroke();
    ctx.fill();
  };
  rect(-11, -14, 2, 27); // booms
  rect(9, -14, 2, 27);
  rect(-20, -4, 40, 6); // wing
  rect(-7, 11, 14, 3.5); // horizontal tail
  // Fuselage with a pointed nose
  ctx.beginPath();
  ctx.moveTo(0, -18 * s);
  ctx.lineTo(2.8 * s, -12 * s);
  ctx.lineTo(2.8 * s, 15 * s);
  ctx.lineTo(-2.8 * s, 15 * s);
  ctx.lineTo(-2.8 * s, -12 * s);
  ctx.closePath();
  ctx.stroke();
  ctx.fill();

  ctx.fillStyle = colors['uav-nose'];
  ctx.beginPath();
  ctx.arc(0, -13 * s, 2 * s, 0, Math.PI * 2);
  ctx.fill();
}

function formatMetres(m) {
  const abs = Math.abs(m);
  if (abs >= 1000) return `${+(m / 1000).toFixed(2)} km`;
  if (abs < 1 && abs > 0) return `${+m.toFixed(2)} m`;
  return `${+m.toFixed(0)} m`;
}
