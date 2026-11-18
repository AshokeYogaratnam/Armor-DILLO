// Armor-DILLO dashboard: render state from /ws, send requests to /api/*.
"use strict";
const $ = (id) => document.getElementById(id);
const pct = (x) => (x == null ? "—" : Math.round(x * 100) + "%");
const num = (x, d = 0, unit = "") => (x == null ? "—" : Number(x).toFixed(d) + unit);


function post(path, body) {
  return fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  }).catch(() => {});
}

function meter(id, v, level) {
  const b = $(id);
  b.style.width = Math.round((v || 0) * 100) + "%";
  b.style.background = v >= 0.6 ? "var(--bad)" : v >= 0.3 ? "var(--warn)" : "var(--ok)";
  $(id + "-n").textContent = v == null ? "—" : v.toFixed(2);
}

function setLevel(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = "level " + cls;
}

function render(s) {

  const m = s.motor || {};
  $("state").textContent = s.state || "…";
  $("state").className = "badge " + (s.state || "");
  $("reason").textContent = s.reason || s.latched || "";

  // Banner: who's in charge
  const banner = $("banner");
  if (s.override && s.override.manual) {
    banner.className = "banner manual";
    banner.textContent = "MANUAL override active: the switch and buttons are in control. Dashboard commands are ignored.";
  } else if (s.latched) {
    banner.className = "banner latched";
    banner.textContent = `Closed for: ${s.latched}. To reopen: MANUAL → press OPEN → AUTO.`;
  } else banner.className = "banner hidden";

  // Coverage
  $("cov").textContent = pct(m.coverage);
  $("cov-fill").style.width = ((m.coverage || 0) * 100) + "%";
  const goalCov = m.goal_coverage;
  $("cov-goal").style.display = goalCov == null ? "none" : "";
  if (goalCov != null) $("cov-goal").style.left = goalCov * 100 + "%";
  $("cov-req").style.display = s.request == null ? "none" : "";
  if (s.request != null) $("cov-req").style.left = s.request * 100 + "%";
  $("source").textContent = s.source || "—";
  const locked = !!(s.override && s.override.manual);
  for (const id of ["open", "close", "set", "slider"]) $(id).disabled = locked;

  // Override switches
  const ov = s.override || {};
  $("ov-mode").textContent = ov.manual ? "MANUAL" : "AUTO";
  $("ov-mode").className = "sw" + (ov.manual ? " on" : "");
  $("ov-open").className = "sw" + (ov.open ? " on" : "");
  $("ov-close").className = "sw" + (ov.close ? " on" : "");

  // Vision
  $("armed").textContent = s.armed ? (s.ready ? "armed" : "arming") : "disarmed";
  $("armed").className = "pill" + (s.armed ? " on" : "");
  const imp = s.impact || {};
  setLevel("impact-level", s.armed ? imp.level : "off", s.armed ? imp.level : "off");
  $("impact-ttc").textContent = imp.ttc == null ? "—" : imp.ttc.toFixed(2) + " s";
  $("impact-area").textContent = imp.area ? imp.area + " px" : "—";
  $("impact-points").textContent = imp.area ? (imp.points || "area") : "—";
  const d = s.dust || {};
  setLevel("dust-level", s.armed ? d.level + (d.duration ? ` · ${d.duration}s` : "") : "off",
           s.armed ? d.level : "off");
  meter("dust-sev", d.severity); meter("dust-c", d.contrast);
  meter("dust-h", d.haze); meter("dust-p", d.particles);
  const cam = s.camera || {};
  const camBad = cam.problem || cam.error;
  setLevel("cam-level", camBad ? (cam.problem || cam.error) : "ok", camBad ? "bad" : "ok");
  $("cam-fps").textContent = `${num(cam.fps, 1)} / ${cam.target_fps}`;
  $("cam-dropped").textContent = num(cam.dropped);
  $("cam-proc").textContent = num(cam.process_ms, 1, " ms");

  // Latency
  const lat = s.latency || {};
  $("lat-live").textContent = num(lat.capture_to_detect_ms, 1, " ms");
  $("lat-thr").textContent = num(lat.ttc_threshold, 2, " s");
  $("lat-close").textContent = lat.close_time == null ? "not calibrated" : lat.close_time + " s";
  const L = lat.last;
  $("lat-last").textContent = !L ? "—" :
    `${L.capture_to_detect_ms || "–"} + ${L.detect_to_command_ms || "–"} ms, close ${L.close_s} s` +
    (L.margin_s ? `, margin ${L.margin_s} s` : "");
  $("lat-last").style.color = L && L.margin_s && parseFloat(L.margin_s) < 0 ? "var(--bad)" : "";

  // Motor
  $("m-mode").textContent = m.fault ? "FAULT" : (m.mode || "—");
  $("m-pos").textContent = m.position == null ? "—" :
    `${m.position} ct · ${num(m.output_revs, 2)} rev` + (m.travel_mm != null ? ` · ${m.travel_mm} mm` : "");
  $("m-vel").textContent = num(m.velocity, 0, " ct/s");
  $("m-duty").textContent = m.duty == null ? "—" : Math.round(m.duty * 100) + "%";
  $("m-pwm").textContent = m.hardware_pwm ? "hardware" : "SOFTWARE (fallback)";
  $("m-health").textContent = `${m.encoder_errors ?? "—"} / ${m.watchdog_trips ?? "—"} / ${m.overruns ?? "—"}`;

  // Events
  const ol = $("events");
  const html = (s.events || []).map((e) =>
    `<li><span class="t">${e.t}</span><span class="${e.level}">${e.level}</span><span>${escape(e.text)}</span></li>`).join("");
  if (ol.dataset.html !== html) { ol.innerHTML = html; ol.dataset.html = html; }
}

function escape(t) {
  return String(t).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[c]);
}

// WebSocket with reconnect
function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { $("conn").textContent = "live"; $("conn").className = "conn up"; };
  ws.onmessage = (ev) => { try { render(JSON.parse(ev.data)); } catch (e) { console.error(e); } };
  ws.onclose = () => {
    $("conn").textContent = "disconnected"; $("conn").className = "conn";
    setTimeout(connect, 1000);
  };
}

// MJPEG preview with retry
function startStream() {
  const img = $("stream");
  img.onload = () => $("nocam").classList.add("hidden");
  img.onerror = () => { $("nocam").classList.remove("hidden"); setTimeout(startStream, 2000); };
  img.src = "/stream?" + Date.now();
}

$("slider").oninput = () => { $("slider-n").textContent = $("slider").value + "%"; };
$("set").onclick = () => post("/api/coverage", { value: $("slider").value / 100 });
$("open").onclick = () => post("/api/open");
$("close").onclick = () => post("/api/close");
$("emergency").onclick = () => post("/api/emergency");

connect();
startStream();
