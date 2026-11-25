// Rocket simulator, controllers and statistics: the same model as the notebook. No page code here.

// ---------- parameters (SI units, radians) ----------
const P = { mFuel: 100, Tmax: 18000, Ts: 400, ls: 2.5, L: 6, Isp: 220, g0: 9.80665, g: 9.81, rho: 1.225, CdA: 5,
            tauW: 2, dt: 0.1, maxSteps: 400, xLim: 150, yLim: 200 };
const LIMITS = { vy: 2, vx: 1, tilt: 10 * Math.PI / 180, pad: 10 };
const ACTIONS = [[0, 0], [0, 1], [0, -1], [1, 0], [1, 1], [1, -1]];       // (main engine, side thruster)
const OUTCOMES = ["flying", "landed", "crashed", "out of area", "out of fuel", "time limit"];
const FLYING = 0, LANDED = 1, CRASHED = 2, OUT_OF_AREA = 3, OUT_OF_FUEL = 4, TIMEOUT = 5;

// test scenarios: [low, high] ranges drawn for every episode
const SCENARIOS = {
  calm:     { wind: [0, 0],   gust: [0, 0], m0: [1000, 1000], g: [9.81, 9.81] },
  training: { wind: [0, 8],   gust: [0, 2], m0: [900, 1100],  g: [9.78, 9.83] },
  unseen:   { wind: [10, 15], gust: [3, 5], m0: [1000, 1000], g: [9.81, 9.81] },
};

// ---------- random numbers ----------
function mulberry32(a) {
  return function () {
    a |= 0; a = a + 0x6D2B79F5 | 0;
    let t = Math.imul(a ^ a >>> 15, 1 | a);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}
// independent stream for episode i of a batch, the same for every controller
function episodeRng(seed, i) {
  let h = Math.imul(seed ^ 0x9E3779B9, 0x85EBCA6B) ^ Math.imul(i + 1, 0xC2B2AE35);
  h = Math.imul(h ^ h >>> 16, 0x85EBCA6B);
  return mulberry32(h ^ h >>> 13);
}
const uniform = (rng, [lo, hi]) => lo + (hi - lo) * rng();
function normal(rng) {                                    // Box-Muller
  return Math.sqrt(-2 * Math.log(1 - rng())) * Math.cos(2 * Math.PI * rng());
}

// ---------- dynamics: state s = [x, y, vx, vy, theta, omega, m] ----------
function deriv(s, main, side, wind, g, mDry) {
  const [, , vx, vy, th, om, m] = s;
  const fuel = m > mDry + 1e-9 ? 1 : 0;
  const T = P.Tmax * main * fuel, S = P.Ts * side * fuel;
  const rx = vx - wind;
  const drag = 0.5 * P.rho * P.CdA * Math.hypot(rx, vy);
  return [vx, vy, (-T * Math.sin(th) - drag * rx) / m, (T * Math.cos(th) - drag * vy) / m - g,
          om, S * P.ls / (m * P.L * P.L / 12), -(T + Math.abs(S)) / (P.Isp * P.g0)];
}
function rk4Step(s, main, side, wind, g, mDry, dt) {
  const f = (z) => deriv(z, main, side, wind, g, mDry);
  const add = (a, k, c) => a.map((v, i) => v + c * k[i]);
  const k1 = f(s), k2 = f(add(s, k1, dt / 2)), k3 = f(add(s, k2, dt / 2)), k4 = f(add(s, k3, dt));
  const out = s.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  out[6] = Math.max(out[6], mDry);
  return out;
}
function touchdownOk(s) {
  return { vy: Math.abs(s[3]) <= LIMITS.vy, vx: Math.abs(s[2]) <= LIMITS.vx,
           tilt: Math.abs(s[4]) <= LIMITS.tilt, pad: Math.abs(s[0]) <= LIMITS.pad };
}

// ---------- one episode ----------
// cond: { wind, gust, m0, g } numbers (wind signed). start: optional fixed [x, y, vx, vy, theta, omega].
function makeEpisode(cond, rng, start) {
  const deg5 = 5 * Math.PI / 180;
  const b = start || [uniform(rng, [-30, 30]), uniform(rng, [80, 120]), uniform(rng, [-3, 3]), uniform(rng, [-12, -6]),
                      uniform(rng, [-deg5, deg5]), uniform(rng, [-0.05, 0.05])];
  return { s: [...b, cond.m0], mDry: cond.m0 - P.mFuel, g: cond.g, wMean: cond.wind, gust: cond.gust,
           w: cond.wind + (cond.gust ? cond.gust * normal(rng) : 0), rng, k: 0, outcome: FLYING };
}
function drawConditions(sc, rng) {
  const sign = rng() < 0.5 ? -1 : 1;
  return { wind: sign * uniform(rng, sc.wind), gust: uniform(rng, sc.gust), m0: uniform(rng, sc.m0), g: uniform(rng, sc.g) };
}
function stepEpisode(ep, action) {
  if (ep.outcome !== FLYING) return;
  const [main, side] = ACTIONS[action];
  const old = ep.s;
  let s = rk4Step(old, main, side, ep.w, ep.g, ep.mDry, P.dt);
  const hit = s[1] <= 0;
  if (hit) {                                              // state at ground contact (linear interpolation)
    const f = old[1] / (old[1] - s[1]);
    s = old.map((v, i) => v + f * (s[i] - v));
  }
  ep.s = s; ep.k++;
  if (hit) { const ok = touchdownOk(s); ep.outcome = ok.vy && ok.vx && ok.tilt && ok.pad ? LANDED : CRASHED; }
  else if (Math.abs(s[0]) > P.xLim || s[1] > P.yLim) ep.outcome = OUT_OF_AREA;
  else if (s[6] <= ep.mDry + 1e-9) ep.outcome = OUT_OF_FUEL;
  else if (ep.k >= P.maxSteps) ep.outcome = TIMEOUT;
  const a = Math.exp(-P.dt / P.tauW);                     // exact Ornstein-Uhlenbeck update
  ep.w = ep.wMean + (ep.w - ep.wMean) * a + (ep.gust ? ep.gust * Math.sqrt(1 - a * a) * normal(ep.rng) : 0);
}

// ---------- controllers: make() returns act(state) -> action index ----------
const G = { kpX: 0.4, kiX: 0.0371, kdX: 1.42, tiltMax: 0.269, tiltLand: 0.0809, yLow: 20, kpTh: 2.61, kdTh: 2.59,
            dead: 0.0587, vTouch: 0.861, aBrake: 1.0, vMax: 9.12 };
function makePID() {
  let integral = 0;
  return (s) => {
    const [x, y, vx, vy, th, om] = s, err = -x;
    const limit = y < G.yLow ? G.tiltLand : G.tiltMax;
    const raw = -(G.kpX * err + G.kiX * integral - G.kdX * vx) / P.g;
    const target = Math.min(Math.max(raw, -limit), limit);
    if (Math.abs(raw) < limit) integral += err * P.dt;
    integral = Math.min(Math.max(integral, -300), 300);
    const u = G.kpTh * (target - th) - G.kdTh * om;
    const side = Math.abs(u) > G.dead ? Math.sign(u) : 0;
    const vRef = -Math.min(Math.sqrt(G.vTouch ** 2 + 2 * G.aBrake * Math.max(y, 0)), G.vMax);
    return 3 * (vy < vRef ? 1 : 0) + (side > 0 ? 1 : side < 0 ? 2 : 0);
  };
}
function decodeFloat32(b64) {
  const bin = atob(b64), bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Float32Array(bytes.buffer);
}
function loadNet(parts) {
  const a = parts.map(decodeFloat32);
  return [0, 1, 2].map((i) => ({ W: a[2 * i], b: a[2 * i + 1], nOut: a[2 * i + 1].length }));
}
function observe(s) {
  const [x, y, vx, vy, th, om, m] = s;
  return Float32Array.from([x / 20, y / 60 - 1, Math.exp(-Math.max(y, 0) / 8), vx / 4, vy / 6, th / 0.2, om / 0.2, (m - 1000) / 100]);
}
function forward(net, x) {                                // two ReLU layers, linear output
  net.forEach(({ W, b, nOut }, l) => {
    const nIn = x.length, out = new Float32Array(nOut);
    for (let i = 0; i < nOut; i++) {
      let v = b[i];
      for (let j = 0, p = i * nIn; j < nIn; j++) v += W[p + j] * x[j];
      out[i] = l < 2 && v < 0 ? 0 : v;
    }
    x = out;
  });
  return x;
}
function makeDQN(net) {
  return (s) => { const q = forward(net, observe(s)); let best = 0; for (let i = 1; i < q.length; i++) if (q[i] > q[best]) best = i; return best; };
}
function makeControllers(weights) {
  const calm = loadNet(weights.calm), robust = loadNet(weights.robust);
  return { pid: makePID, calm: () => makeDQN(calm), robust: () => makeDQN(robust) };
}

// ---------- running episodes and statistics ----------
function runEpisode(ep, act) { while (ep.outcome === FLYING) stepEpisode(ep, act(ep.s)); return ep; }
// episode i of a batch: scenario name, or fixed conditions { wind, gust, m0, g }
function batchEpisode(setup, seed, i) {
  const rng = episodeRng(seed, i);
  return makeEpisode(typeof setup === "string" ? drawConditions(SCENARIOS[setup], rng) : setup, rng);
}
function wilson(k, n, z = 1.96) {
  const p = k / n, d = 1 + z * z / n, mid = p + z * z / (2 * n), h = z * Math.sqrt(p * (1 - p) / n + z * z / (4 * n * n));
  return [(mid - h) / d, (mid + h) / d];
}

// ---------- notebook reference values ----------
const REF_FLIGHT = {                                      // notebook section 5.5: steps, x, vx, vy, tilt, mass
  start: [20, 100, 0, -8, 0, 0], cond: { wind: 0, gust: 0, m0: 1000, g: 9.81 }, tol: 1e-3,
  pid:    [145, -1.896290, -0.070674, -1.075700,  0.002365, 933.236398],
  calm:   [133, -0.974019,  0.210581, -0.333386, -0.006769, 938.550048],
  robust: [112, -3.149960,  0.758419, -0.676912, -0.007576, 947.275455],
};
const REF_RATES = {                                       // notebook sections 5.1 and 5.2: success %, 1,000 landings
  calm:     { pid: 100.0, calm: 99.6, robust: 99.5 },
  training: { pid: 100.0, calm: 51.3, robust: 98.9 },
  unseen:   { pid: 92.1,  calm: 78.7, robust: 59.2 },
};
// agreement test for two independent samples: 3 standard errors, and never tighter than 1 point
function rateTolerance(refPct, n) {
  const p = refPct / 100;
  return Math.max(1, 300 * Math.sqrt(p * (1 - p) * (1 / 1000 + 1 / n)));
}
function checkReferenceFlights(controllers) {
  const out = {};
  for (const name of ["pid", "calm", "robust"]) {
    const ep = runEpisode(makeEpisode(REF_FLIGHT.cond, null, REF_FLIGHT.start), controllers[name]());
    const s = ep.s, got = [ep.k, s[0], s[2], s[3], s[4], s[6]];
    const err = Math.max(...got.map((v, i) => Math.abs(v - REF_FLIGHT[name][i])));
    out[name] = { err, ok: err <= REF_FLIGHT.tol };
  }
  return out;
}

if (typeof module !== "undefined") module.exports = { P, SCENARIOS, REF_RATES, makeControllers, checkReferenceFlights,
  runEpisode, batchEpisode, wilson, rateTolerance, rk4Step, OUTCOMES, LANDED, episodeRng, normal };
