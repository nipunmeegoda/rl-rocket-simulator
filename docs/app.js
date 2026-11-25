// Page logic: flight view, controls, batch runs and the notebook check.
const $ = (id) => document.getElementById(id);
const CONTROLLERS = makeControllers(DQN_WEIGHTS);
const NAMES = { pid: "PID", calm: "DQN, calm", robust: "DQN, randomised" };
const COLORS = { pid: "#2a78d6", calm: "#eb6834", robust: "#1baf7a" };
const ctx = $("view").getContext("2d");
const W = 1000, H = 580, GROUND = 540, SCALE = 4.1;      // pixels per metre
const px = (x) => W / 2 + x * SCALE, py = (y) => GROUND - y * SCALE;

let ep, act, prev, action, trail, running = false, frac = 0, last = 0, batchId = 0, batchOn = false;

// ---------- settings ----------
const seed = () => Math.min(2147483647, Math.max(1, Math.floor(+$("seed").value) || 1));
function setup() {                                        // scenario name, or fixed conditions from the sliders
  const sc = $("scenario").value;
  return sc === "custom" ? { wind: +$("wind").value, gust: +$("gust").value, m0: +$("m0").value, g: +$("g").value } : sc;
}
function showSettings() {
  const signed = (v) => (v > 0 ? "+" : "") + v.toFixed(1);
  if ($("scenario").value !== "custom" && ep) {           // show what this episode drew
    $("wind").value = ep.wMean; $("gust").value = ep.gust; $("m0").value = ep.mDry + P.mFuel; $("g").value = ep.g;
  }
  $("windV").textContent = signed(+$("wind").value) + " m/s";
  $("gustV").textContent = (+$("gust").value).toFixed(2) + " m/s";
  $("m0V").textContent = (+$("m0").value).toFixed(0) + " kg";
  $("gV").textContent = (+$("g").value).toFixed(2) + " m/s²";
  $("speedV").textContent = $("speed").value + "×";
}

// ---------- flight ----------
function reset() {
  stop();
  ep = batchEpisode(setup(), seed(), 0);
  act = CONTROLLERS[$("ctrl").value]();
  prev = ep.s; action = 0; trail = [[ep.s[0], ep.s[1]]]; frac = 0;
  $("start").disabled = false;
  showSettings(); draw();
}
function start() {
  if (ep.outcome !== FLYING) return;
  running = true; last = performance.now(); $("start").textContent = "Pause"; requestAnimationFrame(loop);
}
function stop() { running = false; $("start").textContent = "Start"; }
function loop(now) {
  if (!running) return;
  frac += Math.min((now - last) / 1000, 0.1) * +$("speed").value / P.dt;
  last = now;
  while (frac >= 1 && ep.outcome === FLYING) {
    prev = ep.s; action = act(ep.s); stepEpisode(ep, action);
    trail.push([ep.s[0], ep.s[1]]); frac -= 1;
  }
  if (ep.outcome !== FLYING) { frac = 1; stop(); $("start").disabled = true; }
  draw();
  if (running) requestAnimationFrame(loop);
}

// ---------- drawing ----------
function rocketShape(c, scale, main, side) {              // centre line up, base at the origin, 6 m tall
  c.save(); c.scale(scale, scale);
  if (main) {
    c.fillStyle = "#ff9d2e"; c.beginPath(); c.moveTo(-0.45, 0); c.lineTo(0, 2.6 + Math.random()); c.lineTo(0.45, 0); c.fill();
  }
  if (side) {                                             // +1 turns anticlockwise: jet at the nose pushes left, flame on the right
    const d = side > 0 ? 1 : -1;
    c.fillStyle = "#ff9d2e"; c.beginPath(); c.moveTo(0.5 * d, -5.2); c.lineTo(2 * d, -5); c.lineTo(0.5 * d, -4.8); c.fill();
  }
  c.fillStyle = "#f7f9fb"; c.strokeStyle = "#16202c"; c.lineWidth = 0.15;
  c.beginPath(); c.moveTo(-0.5, 0); c.lineTo(-0.5, -4.8); c.quadraticCurveTo(-0.4, -5.7, 0, -6);
  c.quadraticCurveTo(0.4, -5.7, 0.5, -4.8); c.lineTo(0.5, 0); c.closePath(); c.fill(); c.stroke();
  c.fillStyle = "#1c5cab"; c.fillRect(-0.5, -4.4, 1, 0.6);
  c.strokeStyle = "#16202c"; c.lineWidth = 0.2;
  c.beginPath(); c.moveTo(-0.5, -1); c.lineTo(-1.2, 0.2); c.moveTo(0.5, -1); c.lineTo(1.2, 0.2); c.stroke();
  c.restore();
}
function arrow(x, y, dx, color) {
  ctx.strokeStyle = ctx.fillStyle = color; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y); ctx.stroke();
  if (Math.abs(dx) > 2) {
    const d = Math.sign(dx);
    ctx.beginPath(); ctx.moveTo(x + dx + 9 * d, y); ctx.lineTo(x + dx, y - 7); ctx.lineTo(x + dx, y + 7); ctx.fill();
  }
}
function draw() {
  const f = Math.min(frac, 1), s = prev.map((v, i) => v + f * (ep.s[i] - v)), color = COLORS[$("ctrl").value];
  const sky = ctx.createLinearGradient(0, 0, 0, GROUND);
  sky.addColorStop(0, "#cfe3f7"); sky.addColorStop(1, "#f2f8fd");
  ctx.fillStyle = sky; ctx.fillRect(0, 0, W, GROUND);
  ctx.fillStyle = "#b9c4a8"; ctx.fillRect(0, GROUND, W, H - GROUND);
  ctx.font = "13px system-ui, sans-serif"; ctx.textAlign = "left";
  for (let y = 25; y <= 125; y += 25) {
    ctx.strokeStyle = "rgba(22,32,44,.10)"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, py(y)); ctx.lineTo(W, py(y)); ctx.stroke();
    ctx.fillStyle = "#5a6675"; ctx.fillText(y + " m", 8, py(y) - 4);
  }
  ctx.fillStyle = "#16202c"; ctx.fillRect(px(-LIMITS.pad), GROUND - 2, 2 * LIMITS.pad * SCALE, 8);
  ctx.textAlign = "center"; ctx.fillText("landing pad", px(0), GROUND + 25);

  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash([5, 4]); ctx.beginPath();
  trail.slice(0, Math.max(1, trail.length - 1)).forEach(([x, y], i) => ctx[i ? "lineTo" : "moveTo"](px(x), py(y)));
  ctx.lineTo(px(s[0]), py(s[1])); ctx.stroke(); ctx.setLineDash([]);

  const burning = running || (ep.outcome === FLYING && ep.k > 0), [main, side] = burning ? ACTIONS[action] : [0, 0];
  ctx.save(); ctx.translate(px(s[0]), py(s[1])); ctx.rotate(-s[4]); rocketShape(ctx, SCALE * 1.5, main, side); ctx.restore();

  // wind
  ctx.fillStyle = "#16202c"; ctx.textAlign = "center"; ctx.fillText(`wind ${ep.w >= 0 ? "→" : "←"} ${Math.abs(ep.w).toFixed(1)} m/s`, W / 2, 24);
  arrow(W / 2, 40, ep.w * 7, "#1c5cab");

  // close-up
  const bx = W - 182, by = 12, bs = 170;
  ctx.save(); ctx.beginPath(); ctx.rect(bx, by, bs, bs); ctx.clip();
  ctx.fillStyle = "#eef5fc"; ctx.fillRect(bx, by, bs, bs);
  const cx = bx + bs / 2, cy = by + bs * 0.74, gy = cy + s[1] * 18;
  ctx.fillStyle = "#b9c4a8"; ctx.fillRect(bx, gy, bs, bs);
  ctx.translate(cx, cy); ctx.rotate(-s[4]); rocketShape(ctx, 18, main, side); ctx.restore();
  ctx.strokeStyle = "#5a6675"; ctx.lineWidth = 1; ctx.strokeRect(bx, by, bs, bs);
  ctx.fillStyle = "#16202c"; ctx.textAlign = "left"; ctx.fillText("close-up", bx + 6, by + 16);

  // numbers
  const done = ep.outcome !== FLYING, ok = touchdownOk(s), landed = ep.outcome === LANDED || ep.outcome === CRASHED;
  const put = (id, text, good) => { $(id).textContent = text; $(id).className = "v" + (landed && good !== undefined ? (good ? " ok" : " bad") : ""); };
  put("oT", (ep.k * P.dt).toFixed(1) + " s");
  put("oY", Math.max(s[1], 0).toFixed(1) + " m");
  put("oX", Math.abs(s[0]).toFixed(2) + " m", ok.pad);
  put("oVy", s[3].toFixed(2) + " m/s", ok.vy);
  put("oVx", s[2].toFixed(2) + " m/s", ok.vx);
  put("oTh", (s[4] * 180 / Math.PI).toFixed(1) + "°", ok.tilt);
  put("oF", (s[6] - ep.mDry).toFixed(1) + " kg");
  put("oW", ep.w.toFixed(1) + " m/s");
  const o = $("outcome");
  o.className = ep.outcome === LANDED ? "ok" : done ? "bad" : "";
  if (ep.outcome === LANDED) o.textContent = "✔ Landed: every touchdown limit met.";
  else if (ep.outcome === CRASHED) {
    const why = [!ok.vy && "too fast vertically", !ok.vx && "too fast sideways", !ok.tilt && "tilted too far", !ok.pad && "missed the pad"].filter(Boolean);
    o.textContent = "✖ Crashed: " + why.join(", ") + ".";
  } else o.textContent = done ? `✖ Failed: ${OUTCOMES[ep.outcome]}.` : running ? "Flying…" : ep.k ? "Paused." : "Ready. Press Start.";
}

// ---------- batch ----------
function rowHtml(name, k, n, ref) {
  const cell = (v) => `<td>${v}</td>`, label = `<span class="dot" style="background:${COLORS[name]}"></span>${NAMES[name]}`;
  if (!n) return `<tr>${cell(label)}${cell("–")}${cell("–")}<td class="barcell"><div class="bar"></div></td>${cell(ref === undefined ? "–" : ref.toFixed(1) + "%")}${cell("–")}</tr>`;
  const p = 100 * k / n, [lo, hi] = wilson(k, n).map((v) => 100 * v);
  let agree = "–";
  if (ref !== undefined) {
    const tol = rateTolerance(ref, n), ok = Math.abs(p - ref) <= tol;
    agree = `<span class="${ok ? "ok" : "bad"}">${ok ? "✔ yes" : "✖ no"} (±${tol.toFixed(1)})</span>`;
  }
  return `<tr>${cell(label)}${cell(p.toFixed(1) + "%")}${cell(`${lo.toFixed(1)}–${hi.toFixed(1)}%`)}<td class="barcell"><div class="bar"><i style="width:${p}%;background:${COLORS[name]}"></i><b style="left:${lo}%;width:${hi - lo}%"></b></div></td>${cell(ref === undefined ? "–" : ref.toFixed(1) + "%")}${cell(agree)}</tr>`;
}
function showRows(counts, n) {
  const refs = REF_RATES[$("scenario").value] || {};
  $("rows").innerHTML = ["pid", "calm", "robust"].map((c) => rowHtml(c, counts ? counts[c] : 0, n, refs[c])).join("");
}
function clearBatch() {
  batchId++; batchOn = false; $("batch").textContent = "Run batch"; $("prog").hidden = true;
  showRows(null, 0);
  $("batchNote").textContent = "All three controllers fly the same starts and winds. “Agrees?” allows three standard errors of the difference between two random samples.";
}
function runBatch() {
  if (batchOn) { clearBatch(); return; }
  const id = ++batchId, n = +$("nBatch").value, set = setup(), sd = seed(), counts = { pid: 0, calm: 0, robust: 0 };
  let i = 0;
  batchOn = true; $("batch").textContent = "Stop"; $("prog").hidden = false;
  const chunk = () => {
    if (id !== batchId) return;
    const t = performance.now();
    while (i < n && performance.now() - t < 25) {
      for (const c of ["pid", "calm", "robust"]) if (runEpisode(batchEpisode(set, sd, i), CONTROLLERS[c]()).outcome === LANDED) counts[c]++;
      i++;
    }
    $("prog").value = i / n;
    showRows(counts, i);
    if (i < n) { setTimeout(chunk, 0); return; }
    batchOn = false; $("batch").textContent = "Run batch"; $("prog").hidden = true;
    $("batchNote").textContent = `Finished ${n} landings per controller, seed ${sd}.` +
      (typeof set === "string" ? "" : " The notebook has no reference for custom conditions.");
  };
  chunk();
}

// ---------- wiring ----------
$("start").addEventListener("click", () => { if (running) { stop(); draw(); } else start(); });
$("reset").addEventListener("click", reset);
$("batch").addEventListener("click", runBatch);
$("ctrl").addEventListener("change", reset);
["scenario", "seed"].forEach((id) => $(id).addEventListener("change", () => { reset(); clearBatch(); }));
["wind", "gust", "m0", "g"].forEach((id) => $(id).addEventListener("input", () => { $("scenario").value = "custom"; reset(); clearBatch(); }));
$("nBatch").addEventListener("change", clearBatch);
$("speed").addEventListener("input", showSettings);
const checks = checkReferenceFlights(CONTROLLERS);
$("checks").innerHTML = Object.entries(checks).map(([c, { err, ok }]) =>
  `<tr><td><span class="dot" style="background:${COLORS[c]}"></span>${NAMES[c]}</td><td>${err.toExponential(1)}</td><td class="${ok ? "ok" : "bad"}">${ok ? "✔ matches" : "✖ differs"}</td></tr>`).join("");
reset(); clearBatch();
