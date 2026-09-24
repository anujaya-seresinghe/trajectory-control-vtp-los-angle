// Planned 2D trajectory in the local NED frame (n = x = North, e = y = East).
//
// The path is a C2 cubic spline through the user's anchor points (chord-length parametrised,
// natural ends), so heading and curvature are continuous everywhere and the lateral acceleration
// has no steps. Each segment is stored as a cubic Bézier. Dragging an anchor's handle clamps the
// spline's tangent at that anchor. The rest of the spline is re-solved around it, but the curvature
// can then step at the clamped anchor.

export const SAMPLE_SPACING = 0.5; // [m] distance between uploaded trajectory points
export const DEFAULT_SPEED = 16; // [m/s] constant ground speed along the path

const TABLE_STEPS_PER_SEGMENT = 400; // arc-length lookup resolution
const MIN_CHORD = 1e-3; // [m]

export class TrajectoryPlan {
  constructor() {
    // { n, e, tangent: [dn, de] | null }. The tangent is dr/dt with t = chord length, null = automatic.
    this.anchors = [];
    this.version = 0;
    this._cache = null;
    this._tangents = null;
  }

  get length() {
    return this.anchors.length;
  }

  _changed() {
    this.version++;
    this._cache = null;
    this._tangents = null;
  }

  add(n, e) {
    this.anchors.push({ n, e, tangent: null });
    this._changed();
  }

  move(i, n, e) {
    Object.assign(this.anchors[i], { n, e });
    this._changed();
  }

  remove(i) {
    this.anchors.splice(i, 1);
    this._changed();
  }

  undo() {
    this.anchors.pop();
    this._changed();
  }

  clear() {
    this.anchors = [];
    this._changed();
  }

  isClamped(i) {
    return Boolean(this.anchors[i]?.tangent);
  }

  /** Revert an anchor's handles to automatic (curvature-continuous). */
  resetHandles(i) {
    this.anchors[i].tangent = null;
    this._changed();
  }

  /**
   * Drag handle `side` ('in' | 'out') of anchor i to world position (n, e).
   * The opposite handle mirrors it, so the heading stays continuous.
   */
  setHandle(i, side, n, e) {
    const a = this.anchors[i];
    const h = this._chords();
    const chord = side === 'out' ? h[i] : h[i - 1];
    if (!chord) return;
    const sign = side === 'out' ? 1 : -1;
    const dn = (sign * 3 * (n - a.n)) / chord;
    const de = (sign * 3 * (e - a.e)) / chord;
    if (Math.hypot(dn, de) < 1e-3) return;
    a.tangent = [dn, de];
    this._changed();
  }

  // ---------------------------------------------------------------- geometry

  _chords() {
    const h = [];
    for (let i = 0; i + 1 < this.anchors.length; i++) {
      const a = this.anchors[i];
      const b = this.anchors[i + 1];
      h.push(Math.max(MIN_CHORD, Math.hypot(b.n - a.n, b.e - a.e)));
    }
    return h;
  }

  /** Spline derivatives at every anchor: natural C2 cubic spline with clamped anchors fixed. */
  _solveTangents() {
    if (this._tangents) return this._tangents;
    const pts = this.anchors;
    const m = pts.length;
    if (m < 2) return (this._tangents = pts.map(() => [0, 0]));

    const h = this._chords();
    // Tridiagonal system  lower[i] D[i-1] + diag[i] D[i] + upper[i] D[i+1] = rhs[i]
    const lower = new Float64Array(m);
    const diag = new Float64Array(m);
    const upper = new Float64Array(m);
    const rhs = pts.map(() => [0, 0]);
    const delta = (i) => [pts[i + 1].n - pts[i].n, pts[i + 1].e - pts[i].e];

    for (let i = 0; i < m; i++) {
      if (pts[i].tangent) {
        diag[i] = 1;
        rhs[i] = [...pts[i].tangent];
      } else if (i === 0) {
        // natural end: r''(0) = 0
        diag[i] = 2;
        upper[i] = 1;
        const d = delta(0);
        rhs[i] = [(3 * d[0]) / h[0], (3 * d[1]) / h[0]];
      } else if (i === m - 1) {
        lower[i] = 1;
        diag[i] = 2;
        const d = delta(m - 2);
        rhs[i] = [(3 * d[0]) / h[m - 2], (3 * d[1]) / h[m - 2]];
      } else {
        // continuity of r'' at interior knot i
        const h0 = h[i - 1];
        const h1 = h[i];
        const d0 = delta(i - 1);
        const d1 = delta(i);
        lower[i] = h1;
        diag[i] = 2 * (h0 + h1);
        upper[i] = h0;
        rhs[i] = [3 * ((h1 / h0) * d0[0] + (h0 / h1) * d1[0]), 3 * ((h1 / h0) * d0[1] + (h0 / h1) * d1[1])];
      }
    }

    // Thomas algorithm (diagonally dominant, so no pivoting needed)
    const c = new Float64Array(m);
    const d = pts.map(() => [0, 0]);
    for (let i = 0; i < m; i++) {
      const denom = diag[i] - (i > 0 ? lower[i] * c[i - 1] : 0);
      c[i] = upper[i] / denom;
      for (let k = 0; k < 2; k++) {
        d[i][k] = (rhs[i][k] - (i > 0 ? lower[i] * d[i - 1][k] : 0)) / denom;
      }
    }
    for (let i = m - 2; i >= 0; i--) {
      for (let k = 0; k < 2; k++) d[i][k] -= c[i] * d[i + 1][k];
    }
    return (this._tangents = d);
  }

  /** Absolute handle positions of anchor i; null where the anchor has no such handle. */
  handles(i) {
    const a = this.anchors[i];
    const D = this._solveTangents()[i];
    const h = this._chords();
    return {
      in: i > 0 ? { n: a.n - (D[0] * h[i - 1]) / 3, e: a.e - (D[1] * h[i - 1]) / 3 } : null,
      out: i < this.anchors.length - 1 ? { n: a.n + (D[0] * h[i]) / 3, e: a.e + (D[1] * h[i]) / 3 } : null,
    };
  }

  /** Cubic Bézier control points [[n, e] x 4] for every segment. */
  segments() {
    const segs = [];
    for (let i = 0; i + 1 < this.anchors.length; i++) {
      const a = this.anchors[i];
      const b = this.anchors[i + 1];
      const ha = this.handles(i).out;
      const hb = this.handles(i + 1).in;
      segs.push([
        [a.n, a.e],
        [ha.n, ha.e],
        [hb.n, hb.e],
        [b.n, b.e],
      ]);
    }
    return segs;
  }

  // ---------------------------------------------------------------- sampling

  /**
   * Resample the path every `spacing` metres (plus the end point) at constant `speed`.
   *
   * Per point: x, y (NED), vx, vy = speed * unit tangent (NED), t = s / speed, and the lateral
   * acceleration / jerk as y components in the flight-path frame (x along velocity, y right, z down):
   *   at = V * dgamma/dt = speed^2 * kappa          (positive = turning right)
   *   jt = d(at)/dt      = speed^3 * dkappa/ds
   * where gamma = atan2(vE, vN) is the flight-path heading and kappa = dgamma/ds the signed curvature.
   */
  sample(spacing = SAMPLE_SPACING, speed = DEFAULT_SPEED) {
    if (this._cache && this._cache.spacing === spacing && this._cache.speed === speed) return this._cache.result;

    const segs = this.segments();
    const result = { points: [], length: 0, duration: 0, maxAt: 0, maxJt: 0 };

    if (segs.length) {
      // Arc-length table: cumulative length at uniformly spaced parameter values
      const table = []; // { s, seg, u }
      let s = 0;
      for (let k = 0; k < segs.length; k++) {
        let prev = bezier(segs[k], 0);
        if (k === 0) table.push({ s: 0, seg: 0, u: 0 });
        for (let j = 1; j <= TABLE_STEPS_PER_SEGMENT; j++) {
          const u = j / TABLE_STEPS_PER_SEGMENT;
          const p = bezier(segs[k], u);
          s += Math.hypot(p[0] - prev[0], p[1] - prev[1]);
          table.push({ s, seg: k, u });
          prev = p;
        }
      }
      const total = s;

      const targets = [];
      for (let k = 0; k * spacing <= total + 1e-9; k++) targets.push(k * spacing);
      if (total - targets[targets.length - 1] > 1e-3) targets.push(total);

      let row = 1;
      for (const target of targets) {
        while (row < table.length - 1 && table[row].s < target) row++;
        const lo = table[row - 1];
        const hi = table[row];
        let seg;
        let u;
        if (hi.seg !== lo.seg) {
          // crossing an anchor: the row before belongs to the previous segment at u = 1
          seg = hi.seg;
          u = ((target - lo.s) / Math.max(1e-12, hi.s - lo.s)) * hi.u;
        } else {
          seg = hi.seg;
          const f = (target - lo.s) / Math.max(1e-12, hi.s - lo.s);
          u = lo.u + f * (hi.u - lo.u);
        }
        u = Math.min(1, Math.max(0, u));

        const [x, y] = bezier(segs[seg], u);
        const d1 = bezierD1(segs[seg], u);
        const d2 = bezierD2(segs[seg], u);
        const d3 = bezierD3(segs[seg]);
        const speedU = Math.hypot(d1[0], d1[1]);
        const tn = d1[0] / speedU;
        const te = d1[1] / speedU;

        // Signed curvature kappa = dgamma/ds (NED: positive turning right) and dkappa/ds
        const cross = d1[0] * d2[1] - d1[1] * d2[0];
        const dot = d1[0] * d2[0] + d1[1] * d2[1];
        const kappa = cross / speedU ** 3;
        const dCross = d1[0] * d3[1] - d1[1] * d3[0];
        const dKappaDu = dCross / speedU ** 3 - (3 * cross * dot) / speedU ** 5;
        const dKappaDs = dKappaDu / speedU;

        const at = speed * speed * kappa;
        const jt = speed ** 3 * dKappaDs;
        result.points.push({ x, y, vx: speed * tn, vy: speed * te, at, jt, t: target / speed });
        result.maxAt = Math.max(result.maxAt, Math.abs(at));
        result.maxJt = Math.max(result.maxJt, Math.abs(jt));
      }

      result.length = total;
      result.duration = total / speed;
    }

    this._cache = { spacing, speed, result };
    return result;
  }
}

// ---------------------------------------------------------------- Bézier maths

function unit(n, e) {
  const l = Math.hypot(n, e);
  return l > 1e-9 ? [n / l, e / l] : null;
}

function bezier([p0, p1, p2, p3], u) {
  const v = 1 - u;
  const a = v * v * v;
  const b = 3 * v * v * u;
  const c = 3 * v * u * u;
  const d = u * u * u;
  return [a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]];
}

function bezierD1([p0, p1, p2, p3], u) {
  const v = 1 - u;
  const a = 3 * v * v;
  const b = 6 * v * u;
  const c = 3 * u * u;
  return [
    a * (p1[0] - p0[0]) + b * (p2[0] - p1[0]) + c * (p3[0] - p2[0]),
    a * (p1[1] - p0[1]) + b * (p2[1] - p1[1]) + c * (p3[1] - p2[1]),
  ];
}

function bezierD2([p0, p1, p2, p3], u) {
  const v = 1 - u;
  return [
    6 * v * (p2[0] - 2 * p1[0] + p0[0]) + 6 * u * (p3[0] - 2 * p2[0] + p1[0]),
    6 * v * (p2[1] - 2 * p1[1] + p0[1]) + 6 * u * (p3[1] - 2 * p2[1] + p1[1]),
  ];
}

function bezierD3([p0, p1, p2, p3]) {
  return [6 * (p3[0] - 3 * p2[0] + 3 * p1[0] - p0[0]), 6 * (p3[1] - 3 * p2[1] + 3 * p1[1] - p0[1])];
}
