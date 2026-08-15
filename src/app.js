import { motorcycleConfig } from "../config/motorcycle.js";
import { createController } from "./controllers/index.js";
import { initialState, stepVehicle } from "./physics/vehicle.js";
import { SensorModel } from "./sensors/model.js";
import { evaluateRun } from "./analysis/metrics.js";
import { runSimulation, stateSnapshot } from "./simulation/runner.js";
import { scenarios } from "./simulation/scenarios.js";

const $ = selector => document.querySelector(selector);
const deg = radians => radians * 180 / Math.PI;
const rad = degrees => degrees * Math.PI / 180;
const controls = {
  scenario: $("#scenario"), controller: $("#controller"), throttle: $("#throttle"),
  target: $("#target"), gear: $("#gear"), friction: $("#friction"), delay: $("#delay"),
  noise: $("#noise"), speed: $("#speed"), initialSpeed: $("#initial-speed"),
  controllerHz: $("#controller-hz"), mass: $("#mass"), comHeight: $("#com-height"),
  kp: $("#kp"), kd: $("#kd")
};

const manual = { title: "Manual rider input", duration: 30, initialSpeed: 12, gear: 1, targetPitch: rad(12), controller: "pid", throttle: () => 0.65 };
controls.scenario.innerHTML = `<option value="manual">Manual — live input</option>` + Object.entries(scenarios)
  .map(([key, value]) => `<option value="${key}">${value.title}</option>`).join("");

let state, config, controller, sensor, controllerOutput, history, playing = false;
let nextControl = 0, nextRecord = 0, lastFrame = performance.now(), accumulator = 0, lastMode = "ground";
let activeScenario = manual;
let controllerDt = 0.01;

function readSetup() {
  config = structuredClone(motorcycleConfig);
  config.tire.friction = Number(controls.friction.value);
  config.engine.transportDelay = Number(controls.delay.value) / 1000;
  config.mass = Number(controls.mass.value);
  config.comHeight = Number(controls.comHeight.value);
  const key = controls.scenario.value;
  activeScenario = key === "manual" ? manual : scenarios[key];
  state = initialState(config, { speed: Number(controls.initialSpeed.value) / 3.6, gear: Number(controls.gear.value), pitch: activeScenario.initialPitch ?? 0 });
  controller = createController(controls.controller.value, { kp: Number(controls.kp.value), kd: Number(controls.kd.value) });
  controllerDt = 1 / Number(controls.controllerHz.value);
  sensor = new SensorModel({ pitchNoise: rad(Number(controls.noise.value)), rateNoise: rad(Number(controls.noise.value) * 3), seed: 42 });
  controllerOutput = { torqueCeiling: Infinity, status: "initializing", computeMs: 0 };
  history = [];
  nextControl = 0; nextRecord = 0; accumulator = 0; lastMode = state.mode;
  $("#events").innerHTML = "";
  logEvent("Simulation initialized");
  updateAll();
}

function simulate(seconds) {
  const dt = 0.002;
  accumulator += Math.min(seconds, 0.1) * Number(controls.speed.value);
  while (accumulator >= dt) {
    const targetPitch = rad(Number(controls.target.value));
    const throttle = controls.scenario.value === "manual" ? Number(controls.throttle.value) / 100 : activeScenario.throttle(state.time);
    if (state.time >= nextControl - 1e-12) {
      const observation = sensor.sample(state);
      observation.engineTorque = state.engineTorque;
      observation.gear = Number(controls.gear.value);
      controllerOutput = controller.compute(observation, { targetPitch, controllerDt, config });
      nextControl += controllerDt;
    }
    stepVehicle(state, { throttle, gear: Number(controls.gear.value), torqueCeiling: controllerOutput.torqueCeiling }, config, dt);
    if (state.time >= nextRecord - 1e-12) {
      history.push(stateSnapshot(state, controllerOutput, targetPitch));
      if (history.length > 3000) history.shift();
      nextRecord += 0.02;
    }
    if (state.mode !== lastMode) {
      logEvent(state.mode === "wheelie" ? "Front contact lost — wheelie mode" : "Front-wheel touchdown");
      lastMode = state.mode;
    }
    if (state.unsafe && !history.at(-2)?.unsafe) logEvent("Safety envelope exceeded");
    if (state.unsafe) {
      playing = false;
      accumulator = 0;
      break;
    }
    accumulator -= dt;
  }
}

function logEvent(message) {
  const item = document.createElement("li");
  item.innerHTML = `<time>${state?.time.toFixed(2) ?? "0.00"} s</time><span>${message}</span>`;
  $("#events").prepend(item);
  while ($("#events").children.length > 12) $("#events").lastElementChild.remove();
}

function frame(now) {
  const elapsed = (now - lastFrame) / 1000;
  lastFrame = now;
  if (playing) simulate(elapsed);
  updateAll();
  requestAnimationFrame(frame);
}

function updateAll() {
  $("#sim-clock").textContent = `t = ${state.time.toFixed(2)} s`;
  $("#run-label").textContent = playing ? "Running" : "Paused";
  $(".run-state").classList.toggle("live", playing);
  $("#play").textContent = playing ? "❚❚ Pause" : "▶ Run";
  $("#throttle-value").textContent = `${controls.throttle.value}%`;
  $("#target-value").textContent = `${Number(controls.target.value).toFixed(1)}°`;
  const badge = $("#mode-badge");
  badge.textContent = state.mode === "wheelie" ? "FRONT AIRBORNE" : "BOTH WHEELS";
  badge.classList.toggle("danger", state.unsafe);
  $("#safety-label").textContent = state.unsafe ? "Outside declared model envelope" : "Model envelope nominal";
  $("#safety-label").style.color = state.unsafe ? "var(--red)" : "var(--green)";
  drawScene(); drawChart($("#pitch-chart"), "pitch"); drawChart($("#torque-chart"), "torque"); updateTelemetry(); updateMetrics();
}

function updateTelemetry() {
  const values = [
    ["Pitch", deg(state.pitch).toFixed(2), "deg"], ["Pitch rate", deg(state.pitchRate).toFixed(1), "deg/s"],
    ["Speed", (state.speed * 3.6).toFixed(1), "km/h"], ["Engine", Math.round(state.engineRpm), "rpm"],
    ["Rider request", state.requestedTorque.toFixed(1), "N·m"], ["Permitted", state.permittedTorque.toFixed(1), "N·m"],
    ["Rear slip", (state.slip * 100).toFixed(1), "%"], ["Front clearance", (state.frontClearance * 1000).toFixed(0), "mm"]
  ];
  $("#telemetry").innerHTML = values.map(([label, value, unit]) => `<div class="telemetry-card"><span>${label}</span><strong>${value}</strong> <small>${unit}</small></div>`).join("");
}

function updateMetrics() {
  if (history.length < 10) return;
  const metrics = evaluateRun({ history, scenario: { targetPitch: rad(Number(controls.target.value)) } });
  const classification = $("#classification");
  classification.textContent = metrics.classification.toUpperCase();
  classification.className = `badge ${metrics.classification.includes("failure") ? "danger" : ""}`;
  const values = [["Pitch RMSE", metrics.pitchRmseDeg.toFixed(2), "°"], ["Max overshoot", metrics.maxOvershootDeg.toFixed(2), "°"], ["Peak pitch rate", metrics.maxPitchRateDegS.toFixed(1), "°/s"], ["Within ±2°", metrics.timeWithinTolerance.toFixed(2), "s"], ["Intervention RMS", metrics.interventionRmsNm.toFixed(1), "N·m"], ["Violations", metrics.constraintViolations, "samples"]];
  $("#metrics").innerHTML = values.map(([label,value,unit]) => `<div class="metric"><span>${label}</span><strong>${value}</strong> <small>${unit}</small></div>`).join("");
}

function canvasSize(canvas) {
  const ratio = devicePixelRatio || 1, rect = canvas.getBoundingClientRect();
  if (canvas.width !== Math.round(rect.width * ratio) || canvas.height !== Math.round(rect.height * ratio)) { canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio); }
  const ctx = canvas.getContext("2d"); ctx.setTransform(ratio,0,0,ratio,0,0); return { ctx, width: rect.width, height: rect.height };
}

function drawScene() {
  const { ctx, width, height } = canvasSize($("#scene"));
  ctx.clearRect(0,0,width,height); const ground = height * .75; const scale = Math.min(width / 3.7, height / 1.55); const rearX = width * .41; const wheelR = config.rearWheelRadius * scale;
  ctx.strokeStyle="#24323b";ctx.lineWidth=1; for(let x=0;x<width;x+=42){ctx.beginPath();ctx.moveTo(x,ground);ctx.lineTo(x+22,ground+12);ctx.stroke()} ctx.strokeStyle="#53636e";ctx.beginPath();ctx.moveTo(0,ground);ctx.lineTo(width,ground);ctx.stroke();
  ctx.save();ctx.translate(rearX,ground-wheelR);ctx.rotate(-state.pitch);
  const L=config.wheelbase*scale, frontX=L; const drawWheel=(x,air=false)=>{ctx.save();ctx.translate(x,0);ctx.strokeStyle=air?"#657680":"#d5dde2";ctx.lineWidth=7;ctx.beginPath();ctx.arc(0,0,wheelR,0,Math.PI*2);ctx.stroke();ctx.strokeStyle="#4a5a64";ctx.lineWidth=1;for(let a=0;a<Math.PI*2;a+=Math.PI/4){ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(Math.cos(a)*wheelR*.82,Math.sin(a)*wheelR*.82);ctx.stroke()}ctx.restore()};
  drawWheel(0,false);drawWheel(frontX,state.mode==="wheelie");
  ctx.strokeStyle="#e7edf0";ctx.lineWidth=5;ctx.lineCap="round";ctx.beginPath();ctx.moveTo(0,-4);ctx.lineTo(L*.43,-wheelR*.75);ctx.lineTo(L*.78,-wheelR*.55);ctx.lineTo(frontX,0);ctx.stroke();
  ctx.strokeStyle="#ff6b2c";ctx.lineWidth=9;ctx.beginPath();ctx.moveTo(L*.28,-wheelR*.33);ctx.lineTo(L*.52,-wheelR*1.5);ctx.lineTo(L*.75,-wheelR*.7);ctx.lineTo(L*.37,-wheelR*.72);ctx.closePath();ctx.stroke();
  ctx.fillStyle="#202c34";ctx.beginPath();ctx.ellipse(L*.49,-wheelR*.82,wheelR*.65,wheelR*.38,-.05,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#7f919d";ctx.lineWidth=2;ctx.stroke();
  ctx.fillStyle="#ff6b2c";ctx.beginPath();ctx.moveTo(L*.46,-wheelR*1.4);ctx.quadraticCurveTo(L*.66,-wheelR*1.85,L*.78,-wheelR*1.2);ctx.lineTo(L*.67,-wheelR*.85);ctx.closePath();ctx.fill();
  ctx.fillStyle="#b8c6cf";ctx.beginPath();ctx.arc(L*.48,-wheelR*2.25,wheelR*.25,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#8fa0ad";ctx.lineWidth=8;ctx.beginPath();ctx.moveTo(L*.45,-wheelR*2);ctx.lineTo(L*.34,-wheelR*1.35);ctx.lineTo(L*.58,-wheelR*1.05);ctx.stroke();
  const comX=config.comFromRear*scale,comY=-config.comHeight*scale;ctx.fillStyle="#62d4e8";ctx.beginPath();ctx.arc(comX,comY,6,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#62d4e8";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(comX-13,comY);ctx.lineTo(comX+13,comY);ctx.moveTo(comX,comY-13);ctx.lineTo(comX,comY+13);ctx.stroke();
  ctx.restore();
  const target=rad(Number(controls.target.value));ctx.save();ctx.translate(rearX,ground-wheelR);ctx.rotate(-target);ctx.strokeStyle="#62d4e8aa";ctx.setLineDash([6,7]);ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(scale*1.05,0);ctx.stroke();ctx.restore();
  if(state.tireForce>20){ctx.strokeStyle="#5fe0a0";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(rearX,ground+5);ctx.lineTo(rearX+Math.min(120,state.tireForce/15),ground+5);ctx.stroke();}
  ctx.fillStyle="#8fa0ad";ctx.font="11px Consolas";ctx.fillText(`x ${state.x.toFixed(1)} m`,18,25);ctx.fillText(`Fx ${state.tireForce.toFixed(0)} N`,18,43);
}

function drawChart(canvas, kind) {
  const {ctx,width,height}=canvasSize(canvas);ctx.clearRect(0,0,width,height);const pad={l:44,r:16,t:14,b:28};ctx.strokeStyle="#26343d";ctx.lineWidth=1;for(let i=0;i<5;i++){const y=pad.t+(height-pad.t-pad.b)*i/4;ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(width-pad.r,y);ctx.stroke()}
  const data=history.slice(-800);if(data.length<2)return;const minT=data[0].time,maxT=data.at(-1).time||1;let series,yMin,yMax;
  if(kind==="pitch"){series=[["#ff6b2c",d=>deg(d.pitch)],["#62d4e8",d=>deg(d.targetPitch)]];yMin=Math.min(-2,...data.map(d=>deg(d.pitch)));yMax=Math.max(15,...data.map(d=>deg(d.pitch)),...data.map(d=>deg(d.targetPitch)));}
  else{series=[["#f7c552",d=>d.requestedTorque],["#62d4e8",d=>d.permittedTorque],["#ff6b2c",d=>d.engineTorque]];yMin=Math.min(-30,...data.map(d=>d.engineTorque));yMax=Math.max(120,...data.map(d=>d.requestedTorque));}
  const x=d=>pad.l+(d.time-minT)/Math.max(.01,maxT-minT)*(width-pad.l-pad.r),y=v=>height-pad.b-(v-yMin)/(yMax-yMin)*(height-pad.t-pad.b);for(const[color,get]of series){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();data.forEach((d,i)=>i?ctx.lineTo(x(d),y(get(d))):ctx.moveTo(x(d),y(get(d))));ctx.stroke()}ctx.fillStyle="#748894";ctx.font="10px Consolas";ctx.fillText(yMax.toFixed(0),6,pad.t+4);ctx.fillText(yMin.toFixed(0),6,height-pad.b);ctx.fillText(`${Math.max(0,maxT-minT).toFixed(1)} s window`,width-90,height-8);
}

function applyScenario() {
  const key=controls.scenario.value;activeScenario=key==="manual"?manual:scenarios[key];controls.controller.value=activeScenario.controller;controls.gear.value=String(activeScenario.gear);controls.target.value=String(deg(activeScenario.targetPitch));controls.initialSpeed.value=String(Math.round((activeScenario.initialSpeed??12)*3.6));controls.friction.value=String(activeScenario.friction??1.15);controls.delay.value=String((activeScenario.engineDelay??0)*1000);controls.noise.value=String(deg(activeScenario.sensor?.pitchNoise??0));controls.comHeight.value=String(activeScenario.comHeight??motorcycleConfig.comHeight);readSetup();
}

function download(name,content,type){const link=document.createElement("a");link.href=URL.createObjectURL(new Blob([content],{type}));link.download=name;link.click();URL.revokeObjectURL(link.href)}
function csv(){const keys=Object.keys(history[0]??{}).filter(k=>k!=="violations");return [keys.join(","),...history.map(row=>keys.map(k=>JSON.stringify(row[k])).join(","))].join("\n")}

$("#play").onclick=()=>playing=!playing;$("#step").onclick=()=>{simulate(.02);updateAll()};$("#restart").onclick=readSetup;controls.scenario.onchange=applyScenario;
for(const id of ["throttle","target"])controls[id].oninput=()=>updateAll();for(const id of ["controller","gear","friction","delay","noise","initialSpeed","controllerHz","mass","comHeight","kp","kd"])controls[id].onchange=readSetup;
$("#export-csv").onclick=()=>download("wheelie-run.csv",csv(),"text/csv");$("#export-json").onclick=()=>download("wheelie-run.json",JSON.stringify({metadata:{model:config.identity,controller:controls.controller.value},history},null,2),"application/json");
$("#compare").onclick=()=>{const scenario=controls.scenario.value==="manual"?{...manual,duration:6,initialSpeed:Number(controls.initialSpeed.value)/3.6,throttle:()=>Number(controls.throttle.value)/100}:activeScenario;const rows=["none","threshold","pid","delay-aware","nmpc"].map(name=>{const run=runSimulation(scenario,{controller:name,config,duration:Math.min(7,scenario.duration),controllerDt,targetPitch:rad(Number(controls.target.value)),controllerOptions:{kp:Number(controls.kp.value),kd:Number(controls.kd.value)}});return{name,metrics:evaluateRun(run)}});$("#comparison-body").innerHTML=rows.map(({name,metrics:m})=>`<tr><td>${name}</td><td>${m.classification}</td><td>${m.pitchRmseDeg.toFixed(2)}°</td><td>${m.maxOvershootDeg.toFixed(2)}°</td><td>${m.maxPitchRateDegS.toFixed(1)}°/s</td><td>${m.interventionRmsNm.toFixed(1)} N·m</td><td>${m.controllerMeanComputeMs.toFixed(3)} ms</td></tr>`).join("");$("#comparison-panel").hidden=false;$("#comparison-panel").scrollIntoView({behavior:"smooth"})};
$("#close-comparison").onclick=()=>$("#comparison-panel").hidden=true;

readSetup();
const query = new URLSearchParams(location.search);
if (query.get("demo") === "1") {
  for (let index = 0; index < 250; index += 1) simulate(0.02);
}
if (query.get("autoplay") === "1") playing = true;
requestAnimationFrame(frame);

