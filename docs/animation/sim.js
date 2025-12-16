let cfg = null;
let connectionMap = {};
let movingState = "HOLD";

function clamp(n, min, max) { return Math.max(min, Math.min(max, n)); }

function wrap360(angle) {
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  return angle;
}

function createPins(container, count) {
  if (!container) return;
  container.innerHTML = "";
  for (let i = 0; i < count; i++) container.appendChild(document.createElement("span"));
}

function setElBoxPx(el, x, y, w, h) {
  el.style.left = `${x}px`;
  el.style.top = `${y}px`;
  if (w != null) el.style.width = `${w}px`;
  if (h != null) el.style.height = `${h}px`;
}

function pctToPxX(pct, w) { return (pct / 100) * w; }
function pctToPxY(pct, h) { return (pct / 100) * h; }

function getRightPanelSize() {
  const rp = document.getElementById("rightPanel");
  return { w: rp.clientWidth, h: rp.clientHeight };
}

function placeByPct(el, posPct, center = false) {
  const { w, h } = getRightPanelSize();
  let x = pctToPxX(posPct.x, w);
  let y = pctToPxY(posPct.y, h);

  if (center) {
    const ew = el.offsetWidth || 0;
    const eh = el.offsetHeight || 0;
    x -= ew / 2;
    y -= eh / 2;
  }
  setElBoxPx(el, x, y);
}

function applyShaftGeometry(shaftEl, length, anchor) {
  shaftEl.style.height = `${length}px`;

  let ax = 50, ay = 50;
  let origin = "center bottom";
  let translate = "translate(-50%, -100%)";

  switch (anchor) {
    case "top":    ax = 50; ay = 0;   origin = "center bottom"; translate = "translate(-50%, 0%)"; break;
    case "bottom": ax = 50; ay = 100; origin = "center bottom"; translate = "translate(-50%, -100%)"; break;
    case "left":   ax = 0;  ay = 50;  origin = "center bottom"; translate = "translate(0%, -100%)"; break;
    case "right":  ax = 100;ay = 50;  origin = "center bottom"; translate = "translate(-100%, -100%)"; break;
    case "center":
    default:       ax = 50; ay = 50;  origin = "center bottom"; translate = "translate(-50%, -100%)";
  }

  shaftEl.style.left = `${ax}%`;
  shaftEl.style.top = `${ay}%`;
  shaftEl.style.transformOrigin = origin;
  shaftEl.dataset.translate = translate;
}

function rotateShaft(shaftEl, angleDeg) {
  const t = shaftEl.dataset.translate || "translate(-50%, -100%)";
  shaftEl.style.transform = `${t} rotate(${angleDeg}deg)`;
}

function setEncoderLabel(text) {
  const el = document.getElementById("encLabel");
  if (el) el.textContent = text || "Quad Encoder";
}

function setMonitorValues() {
  const pidPill = document.getElementById("pidPill");
  pidPill.textContent = cfg.pid?.enabled ? "PID ON" : "PID OFF";
  pidPill.classList.toggle("off", !cfg.pid?.enabled);

  document.getElementById("tgtAngle").textContent = Math.round(cfg.motion.targetAngle);
  document.getElementById("spd").textContent = Math.round(cfg.motion.speedDegPerSec);
  document.getElementById("dir").textContent = cfg.motion.direction.toUpperCase();

  document.getElementById("mbId").textContent = cfg.modbus.id;
  document.getElementById("mbIp").textContent = cfg.modbus.ip;
  document.getElementById("mbBaud").textContent = cfg.modbus.baud;

  const run = clamp(cfg.pwm.runningDuty, 0, 100);
  const hold = clamp(cfg.pwm.holdingDuty, 0, 100);

  document.getElementById("pwmRun").style.width = `${run}%`;
  document.getElementById("pwmHold").style.width = `${hold}%`;
  document.getElementById("pwmRunVal").textContent = `${run}%`;
  document.getElementById("pwmHoldVal").textContent = `${hold}%`;
  document.getElementById("mbRunDuty").textContent = `${run}`;
}

function setScope() {
  const scope = document.getElementById("pwmScope");
  const wave = document.getElementById("scopeWave");
  const title = document.getElementById("scopeTitle");
  const dutyEl = document.getElementById("scopeDuty");

  const enabled = !!cfg.layout.pwmScope?.enabled;
  scope.style.display = enabled ? "block" : "none";
  if (!enabled) return;

  const { w, h } = getRightPanelSize();
  const pos = cfg.layout.pwmScope.posPct;
  const size = cfg.layout.pwmScope.sizePct;

  const x = pctToPxX(pos.x, w);
  const y = pctToPxY(pos.y, h);
  const sw = pctToPxX(size.w, w);
  const sh = pctToPxY(size.h, h);
  setElBoxPx(scope, x, y, sw, sh);

  title.textContent = cfg.layout.pwmScope.title || "PWM OUTPUT";
  const duty = clamp(cfg.pwm.runningDuty, 0, 100);
  dutyEl.textContent = `${duty}%`;

  const period = clamp(cfg.pwm.periodPx ?? 64, 24, 140);
  wave.style.setProperty("--duty", `${duty}`);
  wave.style.setProperty("--period", `${period}px`);
}

function drawConnections() {
  const svg = document.getElementById("wires");
  const { w, h } = getRightPanelSize();

  svg.setAttribute("width", w);
  svg.setAttribute("height", h);
  svg.innerHTML = "";
  connectionMap = {};

  const defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
  defs.innerHTML = `
    <marker id="arrowPower" class="marker-power" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
      <path d="M0,0 L6,3 L0,6 Z"></path>
    </marker>
    <marker id="arrowFeedback" class="marker-feedback" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
      <path d="M0,0 L6,3 L0,6 Z"></path>
    </marker>
    <marker id="arrowPwm" class="marker-pwm" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
      <path d="M0,0 L6,3 L0,6 Z"></path>
    </marker>
    <marker id="arrowLogic" class="marker-logic" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
      <path d="M0,0 L6,3 L0,6 Z"></path>
    </marker>
  `;
  svg.appendChild(defs);

  const markerByType = (type) => {
    if (type === "feedback") return "url(#arrowFeedback)";
    if (type === "pwm") return "url(#arrowPwm)";
    if (type === "logic") return "url(#arrowLogic)";
    return "url(#arrowPower)";
  };

  (cfg.connections || []).forEach((conn) => {
    const ptsPct = conn.pointsPct || [];
    if (ptsPct.length < 2) return;

    const pts = ptsPct.map(([px, py]) => [pctToPxX(px, w), pctToPxY(py, h)]);
    let d = `M ${pts[0][0]} ${pts[0][1]}`;
    for (let i = 1; i < pts.length; i++) d += ` L ${pts[i][0]} ${pts[i][1]}`;

    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("d", d);
    path.setAttribute("class", `wire ${conn.type || "power"}`);
    path.dataset.id = conn.id;

    if (conn.color) path.style.stroke = conn.color;

    const marker = markerByType(conn.type);
    const arrow = conn.arrow || "end";
    if (arrow === "start" || arrow === "both") path.setAttribute("marker-start", marker);
    if (arrow === "end" || arrow === "both") path.setAttribute("marker-end", marker);

    svg.appendChild(path);
    connectionMap[conn.id] = path;
  });

  applyMotorTypeVisibility();
}

function layoutRightPanelElements() {
  const dc = document.getElementById("dcMotor");
  const st = document.getElementById("stepperMotor");
  const enc = document.getElementById("encoder");
  const chip = document.getElementById("mcu");

  dc.style.display = cfg.layout.dcMotor.enabled === false ? "none" : "block";
  st.style.display = cfg.layout.stepperMotor.enabled === false ? "none" : "block";

  placeByPct(dc, cfg.layout.dcMotor.posPct, false);
  placeByPct(st, cfg.layout.stepperMotor.posPct, false);

  placeByPct(enc, cfg.layout.encoder.posPct, false);
  setEncoderLabel(cfg.layout.encoder.label);

  const chipSize = cfg.layout.mcu.bodySize ?? 120;
  setElBoxPx(chip, 0, 0, chipSize, chipSize);
  const { w, h } = getRightPanelSize();
  const cx = pctToPxX(cfg.layout.mcu.posPct.x, w);
  const cy = pctToPxY(cfg.layout.mcu.posPct.y, h);
  setElBoxPx(chip, cx - chipSize/2, cy - chipSize/2, chipSize, chipSize);

  const nPins = clamp(cfg.layout.mcu.pinsPerSide ?? 7, 3, 14);
  createPins(chip.querySelector(".pins--top"), nPins);
  createPins(chip.querySelector(".pins--bottom"), nPins);
  createPins(chip.querySelector(".pins--left"), nPins);
  createPins(chip.querySelector(".pins--right"), nPins);

  const dcShaft = dc.querySelector(".shaft");
  const stShaft = st.querySelector(".shaft");
  applyShaftGeometry(dcShaft, cfg.layout.dcMotor.shaft.length, cfg.layout.dcMotor.shaft.anchor);
  applyShaftGeometry(stShaft, cfg.layout.stepperMotor.shaft.length, cfg.layout.stepperMotor.shaft.anchor);

  const encOuter = enc.querySelector(".enc-outer");
  const encInner = enc.querySelector(".enc-inner");
  const outerD = (cfg.layout.encoder.outerRadius ?? 36) * 2;
  const innerD = (cfg.layout.encoder.innerRadius ?? 14) * 2;

  encOuter.style.width = `${outerD}px`;
  encOuter.style.height = `${outerD}px`;
  encInner.style.width = `${innerD}px`;
  encInner.style.height = `${innerD}px`;
  encInner.style.top = `${(outerD - innerD) / 2}px`;
  encInner.style.left = `${(outerD - innerD) / 2}px`;
}

function motorRelevantConnectionIds(motorType) {
  if (motorType === "dc") return new Set(["mcu_bts", "bts_dc"]);
  return new Set(["mcu_dmt", "dmt_stepper"]);
}

function applyMotorTypeVisibility() {
  const type = cfg.ui?.motorType || "dc";
  const relevant = motorRelevantConnectionIds(type);

  // fade/hide non-selected motor connections
  Object.keys(connectionMap).forEach((id) => {
    const el = connectionMap[id];
    if (!el) return;

    // always keep encoder_to_mcu and pwm and any "common" lines visible
    const common = (id === "encoder_to_mcu" || id === "pwm");
    if (common) { el.style.opacity = "1"; return; }

    el.style.opacity = relevant.has(id) ? "1" : "0.06";
  });

  // dim the inactive motor visuals
  const dc = document.getElementById("dcMotor");
  const st = document.getElementById("stepperMotor");
  if (type === "dc") {
    dc.classList.remove("dimmed");
    st.classList.add("dimmed");
  } else {
    st.classList.remove("dimmed");
    dc.classList.add("dimmed");
  }

  // update live label
  document.getElementById("mbMotorLive").textContent = (type === "dc") ? "DC" : "STEPPER";
}

function setpointToTargetDeg(sp) {
  // 0–1000 -> 0–359 (never 360)
  sp = clamp(sp, 0, 1000);
  return Math.round((sp / 1000) * 359);
}

function targetDegToSetpoint(deg) {
  deg = clamp(deg, 0, 359);
  return Math.round((deg / 359) * 1000);
}

function stepTowardTarget(angle, target, direction, stepDeg) {
  angle = wrap360(angle);
  target = wrap360(target);

  if (direction === "cw") {
    const dist = (target - angle + 360) % 360;
    if (dist <= stepDeg) return { angle: target, done: true };
    return { angle: wrap360(angle + stepDeg), done: false };
  } else {
    const dist = (angle - target + 360) % 360;
    if (dist <= stepDeg) return { angle: target, done: true };
    return { angle: wrap360(angle - stepDeg), done: false };
  }
}

function updateModbusPanelFromCfg() {
  document.getElementById("mbMotorType").value = cfg.ui.motorType;
  document.getElementById("mbPidEnable").checked = !!cfg.pid.enabled;
  document.getElementById("mbDirection").value = cfg.motion.direction;
  document.getElementById("mbSpeed").value = cfg.motion.speedDegPerSec;

  document.getElementById("mbIdInput").value = cfg.modbus.id;
  document.getElementById("mbIpInput").value = cfg.modbus.ip;
  document.getElementById("mbBaudSelect").value = String(cfg.modbus.baud);

  document.getElementById("mbRunDuty").value = cfg.pwm.runningDuty;
  document.getElementById("mbHoldDuty").value = cfg.pwm.holdingDuty;

  const sp = targetDegToSetpoint(cfg.motion.targetAngle);
  document.getElementById("mbSetpoint").value = sp;
  document.getElementById("mbSetpointVal").textContent = sp;
  document.getElementById("mbTargetVal").textContent = Math.round(cfg.motion.targetAngle);

  document.getElementById("mbRunDutyVal").textContent = cfg.pwm.runningDuty;
  document.getElementById("mbHoldDutyVal").textContent = cfg.pwm.holdingDuty;
}

function bindModbusPanel() {
  const mbMotorType = document.getElementById("mbMotorType");
  const mbPidEnable = document.getElementById("mbPidEnable");
  const mbDirection = document.getElementById("mbDirection");
  const mbSpeed = document.getElementById("mbSpeed");
  const mbSetpoint = document.getElementById("mbSetpoint");

  const mbIdInput = document.getElementById("mbIdInput");
  const mbIpInput = document.getElementById("mbIpInput");
  const mbBaudSelect = document.getElementById("mbBaudSelect");

  const mbRunDuty = document.getElementById("mbRunDuty");
  const mbHoldDuty = document.getElementById("mbHoldDuty");

  const mbWrite = document.getElementById("mbWrite");

  // live preview values
  mbSetpoint.addEventListener("input", () => {
    const sp = Number(mbSetpoint.value);
    document.getElementById("mbSetpointVal").textContent = sp;
    document.getElementById("mbTargetVal").textContent = setpointToTargetDeg(sp);
  });

  mbRunDuty.addEventListener("input", () => {
    document.getElementById("mbRunDutyVal").textContent = mbRunDuty.value;
  });

  mbHoldDuty.addEventListener("input", () => {
    document.getElementById("mbHoldDutyVal").textContent = mbHoldDuty.value;
  });

  mbWrite.addEventListener("click", () => {
    // motor type affects connection visibility
    cfg.ui.motorType = mbMotorType.value;

    // runtime control
    cfg.pid.enabled = !!mbPidEnable.checked;
    cfg.motion.direction = mbDirection.value;
    cfg.motion.speedDegPerSec = clamp(Number(mbSpeed.value), 1, 120);

    cfg.motion.targetAngle = setpointToTargetDeg(Number(mbSetpoint.value));

    // modbus params
    cfg.modbus.id = clamp(Number(mbIdInput.value), 1, 247);
    cfg.modbus.ip = String(mbIpInput.value || "192.168.1.50").trim();
    cfg.modbus.baud = Number(mbBaudSelect.value);

    // PWM
    cfg.pwm.runningDuty = clamp(Number(mbRunDuty.value), 0, 100);
    cfg.pwm.holdingDuty = clamp(Number(mbHoldDuty.value), 0, 100);

    // apply everything immediately
    setMonitorValues();
    setScope();
    applyMotorTypeVisibility();

    // button feedback
    mbWrite.textContent = "WRITTEN ✓";
    setTimeout(() => (mbWrite.textContent = "WRITE REGISTERS"), 650);
  });
}

async function main() {
  const res = await fetch("config.json", { cache: "no-store" });
  cfg = await res.json();

  // Ensure ui exists (no crashes)
  if (!cfg.ui) cfg.ui = { motorType: "dc", stopToleranceDeg: 1 };

  // initial render
  setMonitorValues();
  layoutRightPanelElements();
  setScope();
  drawConnections();
  applyMotorTypeVisibility();

  // init modbus panel values + handlers
  updateModbusPanelFromCfg();
  bindModbusPanel();

  window.addEventListener("resize", () => {
    layoutRightPanelElements();
    setScope();
    drawConnections();
    applyMotorTypeVisibility();
  });

  // Simulation: move to target then STOP (hold)
  let angle = cfg.motion.currentAngle ?? 0;
  const tickMs = 20;
  const dt = tickMs / 1000;
  let phase = 0;

  const dcShaft = document.querySelector("#dcMotor .shaft");
  const stShaft = document.querySelector("#stepperMotor .shaft");
  const encInner = document.querySelector("#encoder .enc-inner");
  const curAngleEl = document.getElementById("curAngle");
  const stateEl = document.getElementById("mbState");
  const scopeWave = document.getElementById("scopeWave");

  setInterval(() => {
    const speed = clamp(cfg.motion.speedDegPerSec ?? 30, 1, 120);
    const step = speed * dt;
    const tol = clamp(cfg.ui.stopToleranceDeg ?? 1, 0, 5);

    const target = wrap360(cfg.motion.targetAngle ?? 0);
    const dir = cfg.motion.direction === "cw" ? "cw" : "ccw";

    // move toward target; stop when within tolerance
    const { angle: next, done } = stepTowardTarget(angle, target, dir, step);

    const diff = Math.min((target - angle + 360) % 360, (angle - target + 360) % 360);
    if (diff <= tol) {
      angle = target;
      movingState = "HOLD";
    } else {
      angle = next;
      movingState = done ? "HOLD" : "MOVING";
    }

    // update visuals
    rotateShaft(dcShaft, angle);
    rotateShaft(stShaft, angle);

    if (cfg.layout.encoder.rotateInner) {
      const mult = cfg.layout.encoder.innerSpeedMultiplier ?? 2.0;
      encInner.style.transform = `rotate(${angle * mult}deg)`;
    }

    curAngleEl.textContent = Math.round(angle);
    cfg.motion.currentAngle = angle;

    // oscilloscope scroll
    phase = (phase - 2) % 5000;
    scopeWave.style.setProperty("--phase", `${phase}px`);

    // update monitor values periodically (cheap + correct)
    document.getElementById("tgtAngle").textContent = Math.round(cfg.motion.targetAngle);
    document.getElementById("spd").textContent = Math.round(cfg.motion.speedDegPerSec);
    document.getElementById("dir").textContent = cfg.motion.direction.toUpperCase();
    stateEl.textContent = movingState;
  }, tickMs);
}

main().catch(err => console.error("Simulator failed to start:", err));
