"""
Lightweight web dashboard for the ROS 2 drone stack.

This intentionally uses only the Python standard library plus ROS messages so it
runs on the Raspberry Pi without Node.js, Flask, or extra web dependencies.

Subscribes to the main namespaced status topics and serves:
    http://<pi-ip>:8080/             dashboard HTML
    http://<pi-ip>:8080/api/status   latest status JSON

It can also publish operator requests through:
    POST /api/autonomy_request {"enabled": true|false}
    POST /api/mavsdk_request {"enabled": true|false}
    POST /api/mission_request {"enabled": true|false}
    POST /api/abort_hold {"confirm": true}
    POST /api/land {"confirm": true}
"""

from __future__ import annotations

import copy
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_interfaces.msg import ControlCommand, DetectionArray, DroneTelemetry, MavsdkActionCommand, TargetError


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Drone Autonomy Flight Deck</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #050812;
      --panel: rgba(8, 14, 28, 0.86);
      --panel2: rgba(15, 24, 45, 0.78);
      --line: rgba(99, 179, 237, 0.20);
      --text: #e7f4ff;
      --muted: #83a3bd;
      --cyan: #37e8ff;
      --green: #50fa7b;
      --yellow: #ffcc66;
      --red: #ff5f75;
      --blue: #6aa8ff;
      --violet: #b794f4;
      --battery: 0%;
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #06101e;
      overflow-x: hidden;
    }

    body::before, body::after { display: none; }

    .shell { width: min(1680px, 100%); margin: 0 auto; padding: 20px; position: relative; z-index: 1; }

    header {
      display: grid;
      grid-template-columns: minmax(320px, 1fr) auto;
      gap: 18px;
      align-items: center;
      margin-bottom: 18px;
    }

    .titleBlock {
      border: 1px solid rgba(55, 232, 255, .18);
      border-radius: 22px;
      padding: 20px 22px;
      background: #0a1528;
      box-shadow: none;
      position: relative;
      overflow: hidden;
    }

    .titleBlock::before { display: none; }

    .eyebrow { color: var(--cyan); letter-spacing: .22em; font-size: 12px; font-weight: 800; text-transform: uppercase; }
    h1 { margin: 7px 0 6px; font-size: clamp(30px, 4vw, 56px); line-height: .92; letter-spacing: -.06em; }
    .sub { color: var(--muted); font-size: 14px; max-width: 760px; }

    .topPills { display: flex; flex-wrap: wrap; gap: 10px; justify-content: flex-end; }
    .pill {
      min-width: 132px;
      padding: 12px 14px;
      border-radius: 16px;
      border: 1px solid rgba(255,255,255,.09);
      background: rgba(7, 12, 24, .70);
      box-shadow: none;
    }
    .pill .k { color: var(--muted); font-size: 10px; text-transform: uppercase; letter-spacing: .16em; }
    .pill .v { margin-top: 5px; font-size: 17px; font-weight: 900; }

    main { display: grid; grid-template-columns: 1.5fr .9fr; gap: 18px; }
    .leftStack, .rightStack { display: grid; gap: 18px; }

    .card {
      border: 1px solid rgba(100, 160, 220, .18);
      border-radius: 22px;
      padding: 18px;
      background: var(--panel);
      box-shadow: none;
      position: relative;
      overflow: hidden;
    }

    .card::before { display: none; }

    .card > * { position: relative; z-index: 1; }

    .cardTitle {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      color: var(--muted);
      font-size: 12px;
      letter-spacing: .16em;
      text-transform: uppercase;
      font-weight: 800;
      margin-bottom: 14px;
    }

    .statusLight { display: inline-flex; align-items: center; gap: 7px; }
    .dot { width: 9px; height: 9px; border-radius: 50%; background: currentColor; }
    .ok { color: var(--green); }
    .warn { color: var(--yellow); }
    .bad { color: var(--red); }
    .cyan { color: var(--cyan); }

    .missionHero {
      display: grid;
      grid-template-columns: minmax(260px, 1fr) 220px;
      gap: 14px;
      align-items: stretch;
    }

    .missionState {
      font-size: clamp(34px, 5vw, 70px);
      line-height: .9;
      font-weight: 950;
      letter-spacing: -.07em;
      margin: 6px 0 10px;
      text-shadow: 0 0 24px rgba(55,232,255,.26);
    }

    .missionReason { color: #bad5e8; font-size: 15px; }

    .armButtons { margin-top: 18px; display: flex; flex-wrap: wrap; gap: 10px; }
    button {
      cursor: pointer;
      border: 0;
      border-radius: 14px;
      padding: 13px 16px;
      color: #031018;
      font-weight: 950;
      letter-spacing: .04em;
      text-transform: uppercase;
      box-shadow: none;
    }
    button:hover { filter: brightness(1.08); }
    .enable { background: var(--green); }
    .disable { background: var(--red); color: white; }
    .hold { background: var(--yellow); }
    .pilotButtons button { font-size: 14px; min-width: 150px; }
    .debugDrawer {
      margin-top: 12px;
      border: 1px solid rgba(255,255,255,.10);
      border-radius: 14px;
      padding: 10px 12px;
      background: rgba(255,255,255,.035);
    }
    .debugDrawer summary {
      cursor: pointer;
      color: var(--muted);
      font-weight: 850;
      letter-spacing: .04em;
      text-transform: uppercase;
    }
    .debugDrawer .armButtons { margin-top: 10px; }
    .debugDrawer button { padding: 10px 12px; font-size: 12px; }

    .batteryRing {
      width: 190px;
      height: 190px;
      border-radius: 50%;
      margin: auto;
      background: #07101e;
      display: grid;
      place-items: center;
      border: 10px solid rgba(80,250,123,.45);
      position: relative;
    }
    .batteryRing::before {
      content: "";
      width: 142px;
      height: 142px;
      border-radius: 50%;
      background: #07101e;
      border: 1px solid rgba(255,255,255,.08);
      position: absolute;
    }
    .batteryRingText { position: relative; text-align: center; }
    .batteryRingText .big { font-size: 40px; font-weight: 950; letter-spacing: -.06em; }
    .batteryRingText .small { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: .14em; }

    .hudWrap {
      height: 460px;
      border-radius: 24px;
      overflow: hidden;
      background: #07101e;
      border: 1px solid rgba(55,232,255,.22);
      position: relative;
    }

    .hudGrid {
      position: absolute;
      inset: 0;
      display: none;
    }

    .hudCrosshair {
      position: absolute;
      inset: 0;
    }
    .hudCrosshair::before, .hudCrosshair::after {
      content: "";
      position: absolute;
      background: rgba(55,232,255,.52);
    }
    .hudCrosshair::before { width: 1px; height: 72%; left: 50%; top: 14%; }
    .hudCrosshair::after { height: 1px; width: 72%; top: 50%; left: 14%; }

    .targetBox {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 162px;
      height: 118px;
      border: 2px solid var(--green);
      transform: translate(-50%, -50%);
      will-change: transform;
    }
    .targetBox::before, .targetBox::after {
      content: "";
      position: absolute;
      inset: 10px;
      border: 1px dashed rgba(80,250,123,.38);
    }
    .targetTag {
      position: absolute;
      left: -2px;
      top: -32px;
      background: var(--green);
      color: #04130c;
      padding: 7px 10px;
      border-radius: 9px 9px 9px 0;
      font-size: 12px;
      font-weight: 950;
      letter-spacing: .06em;
    }
    .lost .targetBox { border-color: var(--red); opacity: .62; }
    .lost .targetTag { background: var(--red); color: white; }

    .errorVector {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 2px;
      height: 2px;
      transform-origin: left center;
      border-top: 2px solid var(--cyan);
      will-change: transform, width;
    }
    .errorVector::after {
      content: "";
      position: absolute;
      right: -4px;
      top: -5px;
      width: 9px;
      height: 9px;
      border-right: 2px solid var(--cyan);
      border-top: 2px solid var(--cyan);
      transform: rotate(45deg);
    }

    .hudReadouts {
      position: absolute;
      inset: 18px;
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      pointer-events: none;
    }
    .readoutStack { display: grid; gap: 8px; }
    .readout {
      min-width: 128px;
      padding: 9px 10px;
      border-radius: 12px;
      background: rgba(3, 8, 18, .70);
      border: 1px solid rgba(55,232,255,.18);
    }
    .readout .k { color: var(--muted); font-size: 10px; letter-spacing: .14em; text-transform: uppercase; }
    .readout .v { margin-top: 2px; font-size: 17px; font-weight: 950; }

    .metrics { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; }
    .metric {
      padding: 15px;
      border-radius: 18px;
      background: rgba(255,255,255,.045);
      border: 1px solid rgba(255,255,255,.08);
      min-height: 98px;
    }
    .metric .k { color: var(--muted); text-transform: uppercase; letter-spacing: .12em; font-size: 10px; font-weight: 850; }
    .metric .v { margin-top: 9px; font-size: 28px; font-weight: 950; letter-spacing: -.04em; }
    .metric .s { color: #9fb8ce; font-size: 12px; margin-top: 3px; }

    .instrumentGrid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    canvas { width: 100%; display: block; }
    .instrument canvas { height: 220px; }
    .trace canvas { height: 220px; }

    .gates { display: grid; grid-template-columns: 1fr 1fr; gap: 9px; }
    .preflightList { display: grid; gap: 9px; }
    .gate {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
      padding: 11px 12px;
      border-radius: 14px;
      background: rgba(255,255,255,.045);
      border: 1px solid rgba(255,255,255,.07);
      color: #d9ebfa;
      font-size: 13px;
      font-weight: 800;
    }
    .gate .state { font-size: 11px; letter-spacing: .12em; text-transform: uppercase; }

    .bars { display: grid; gap: 12px; }
    .barRow { display: grid; grid-template-columns: 74px 1fr 80px; gap: 10px; align-items: center; font-size: 13px; color: #cfe6f8; }
    .barTrack { height: 10px; border-radius: 999px; background: rgba(255,255,255,.07); overflow: hidden; position: relative; }
    .barFill { height: 100%; width: 50%; background: var(--cyan); border-radius: 999px; transform-origin: left center; }
    .barValue { text-align: right; color: var(--muted); }

    pre {
      white-space: pre-wrap;
      word-break: break-word;
      max-height: 250px;
      overflow: auto;
      margin: 0;
      color: #b6d8f2;
      font-size: 12px;
      line-height: 1.45;
      background: rgba(0,0,0,.24);
      border-radius: 14px;
      padding: 12px;
      border: 1px solid rgba(255,255,255,.06);
    }

    @media (max-width: 1100px) {
      main { grid-template-columns: 1fr; }
      header { grid-template-columns: 1fr; }
      .topPills { justify-content: flex-start; }
    }
    @media (max-width: 720px) {
      .shell { padding: 12px; }
      .missionHero, .instrumentGrid, .metrics, .gates { grid-template-columns: 1fr; }
      .hudWrap { height: 360px; }
      .batteryRing { width: 160px; height: 160px; }
      .batteryRing::before { width: 120px; height: 120px; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header>
      <section class="titleBlock">
        <div class="eyebrow">PX4 · ROS 2 · MAVSDK</div>
        <h1>Autonomy Flight Deck</h1>
        <div class="sub">Live mission state, target lock, telemetry, safety gates, and yaw-only command output from the drone stack.</div>
      </section>
      <section class="topPills">
        <div class="pill"><div class="k">Link</div><div id="pillLink" class="v bad">NO LINK</div></div>
        <div class="pill"><div class="k">Armed</div><div id="pillArmed" class="v bad">FALSE</div></div>
        <div class="pill"><div class="k">Mode</div><div id="pillMode" class="v warn">UNKNOWN</div></div>
        <div class="pill"><div class="k">Uptime</div><div id="pillUptime" class="v cyan">0.0s</div></div>
      </section>
    </header>

    <main>
      <div class="leftStack">
        <section class="card">
          <div class="cardTitle"><span>Mission Brain</span><span id="missionBadge" class="statusLight warn"><span class="dot"></span>WAITING</span></div>
          <div class="missionHero">
            <div>
              <div id="mission" class="missionState warn">UNKNOWN</div>
              <div id="missionReason" class="missionReason">startup</div>
              <div class="armButtons pilotButtons">
                <button class="enable" onclick="systemReady()">System Ready</button>
                <button class="enable" onclick="startSmartMission()">Start Mission</button>
                <button class="hold" onclick="abortHold()">Abort / Hold</button>
                <button class="disable" onclick="landNow()">Land</button>
              </div>
              <details class="debugDrawer">
                <summary>Debug drawer · raw gate controls</summary>
                <div class="armButtons">
                  <button class="enable" onclick="setAutonomy(true)">Autonomy Request ON</button>
                  <button class="disable" onclick="setAutonomy(false)">Autonomy Request OFF</button>
                  <button class="enable" onclick="setMavsdk(true)">MAVSDK Offboard Request ON</button>
                  <button class="disable" onclick="setMavsdk(false)">MAVSDK Offboard Request OFF</button>
                  <button class="enable" onclick="setMission(true)">Mission Request ON</button>
                  <button class="disable" onclick="setMission(false)">Mission Request OFF</button>
                </div>
              </details>
            </div>
            <div class="batteryRing"><div class="batteryRingText"><div id="batteryBig" class="big">0%</div><div id="batteryVolt" class="small">0.00 V</div></div></div>
          </div>
        </section>

        <section class="card">
          <div class="cardTitle"><span>Vision Targeting HUD</span><span id="targetBadge" class="statusLight warn"><span class="dot"></span>SEARCHING</span></div>
          <div id="hud" class="hudWrap">
            <div class="hudGrid"></div>
            <div class="hudCrosshair"></div>
            <div id="errorVector" class="errorVector"></div>
            <div id="targetBox" class="targetBox"><div id="targetTag" class="targetTag">TARGET</div></div>
            <div class="hudReadouts">
              <div class="readoutStack">
                <div class="readout"><div class="k">Class</div><div id="hudClass" class="v">--</div></div>
                <div class="readout"><div class="k">Confidence</div><div id="hudConfidence" class="v">0.00</div></div>
                <div class="readout"><div class="k">Detections</div><div id="hudDetections" class="v">0</div></div>
                <div class="readout"><div class="k">Distance</div><div id="hudDistance" class="v">--</div></div>
              </div>
              <div class="readoutStack">
                <div class="readout"><div class="k">Error X</div><div id="hudErrX" class="v">0.000</div></div>
                <div class="readout"><div class="k">Error Y</div><div id="hudErrY" class="v">0.000</div></div>
                <div class="readout"><div class="k">Age</div><div id="hudAge" class="v">--</div></div>
              </div>
            </div>
          </div>
        </section>

        <section class="card">
          <div class="cardTitle"><span>Telemetry Strip</span><span id="telemetryAge" class="cyan">--</span></div>
          <div class="metrics">
            <div class="metric"><div class="k">Altitude</div><div id="altitudeMetric" class="v">0.00 m</div><div class="s">relative</div></div>
            <div class="metric"><div class="k">Yaw</div><div id="yawMetric" class="v">0°</div><div class="s">heading</div></div>
            <div class="metric"><div class="k">Satellites</div><div id="satMetric" class="v">0</div><div id="gpsMetric" class="s">GPS unknown</div></div>
            <div class="metric"><div class="k">Velocity</div><div id="velMetric" class="v">0.00</div><div class="s">m/s horizontal</div></div>
          </div>
        </section>

        <section class="card trace">
          <div class="cardTitle"><span>Live Telemetry Traces</span><span>Battery · Altitude · Yaw</span></div>
          <canvas id="traceCanvas"></canvas>
        </section>
      </div>

      <div class="rightStack">
        <section class="card instrument">
          <div class="cardTitle"><span>Artificial Horizon</span><span id="attitudeText" class="cyan">roll 0° · pitch 0°</span></div>
          <canvas id="horizonCanvas"></canvas>
        </section>

        <section class="card instrument">
          <div class="cardTitle"><span>Compass</span><span id="compassText" class="cyan">yaw 0°</span></div>
          <canvas id="compassCanvas"></canvas>
        </section>

        <section class="card">
          <div class="cardTitle"><span>Preflight</span><span id="preflightSummary" class="statusLight warn"><span class="dot"></span>0/7</span></div>
          <div class="preflightList">
            <div class="gate"><span>Telemetry Fresh</span><span id="preflightTelemetry" class="state bad">NO</span></div>
            <div class="gate"><span>PX4 Link</span><span id="preflightLink" class="state bad">NO</span></div>
            <div class="gate"><span>Armed</span><span id="preflightArmed" class="state bad">NO</span></div>
            <div class="gate"><span>Battery</span><span id="preflightBattery" class="state warn">UNKNOWN</span></div>
            <div class="gate"><span>Local Position</span><span id="preflightLocal" class="state bad">NO</span></div>
            <div class="gate"><span>Vision Fresh</span><span id="preflightVision" class="state bad">NO</span></div>
            <div class="gate"><span>Target Lock</span><span id="preflightTarget" class="state warn">LOST</span></div>
          </div>
        </section>

        <section class="card">
          <div class="cardTitle"><span>Safety Gate Matrix</span><span id="gateSummary" class="warn">BLOCKED</span></div>
          <div class="gates">
            <div class="gate"><span>PX4 Link</span><span id="gateLink" class="state bad">NO</span></div>
            <div class="gate"><span>Armed</span><span id="gateArmed" class="state bad">NO</span></div>
            <div class="gate"><span>Offboard</span><span id="gateOffboard" class="state bad">NO</span></div>
            <div class="gate"><span>Autonomy</span><span id="gateAuto" class="state bad">NO</span></div>
            <div class="gate"><span>Target Lock</span><span id="gateTarget" class="state warn">NO</span></div>
            <div class="gate"><span>GPS Health</span><span id="gateGps" class="state warn">NO</span></div>
            <div class="gate"><span>Command</span><span id="gateCommand" class="state bad">IDLE</span></div>
            <div class="gate"><span>MAVSDK Req</span><span id="gateMavsdkReq" class="state bad">OFF</span></div>
            <div class="gate"><span>MAVSDK Status</span><span id="gateMavsdk" class="state warn">UNKNOWN</span></div>
          </div>
        </section>

        <section class="card">
          <div class="cardTitle"><span>Control Output</span><span id="commandMain" class="warn">UNKNOWN</span></div>
          <div class="bars">
            <div class="barRow"><span>Forward</span><div class="barTrack"><div id="barForward" class="barFill"></div></div><span id="valForward" class="barValue">0.000</span></div>
            <div class="barRow"><span>Right</span><div class="barTrack"><div id="barRight" class="barFill"></div></div><span id="valRight" class="barValue">0.000</span></div>
            <div class="barRow"><span>Down</span><div class="barTrack"><div id="barDown" class="barFill"></div></div><span id="valDown" class="barValue">0.000</span></div>
            <div class="barRow"><span>Yaw</span><div class="barTrack"><div id="barYaw" class="barFill"></div></div><span id="valYaw" class="barValue">0.000</span></div>
          </div>
          <div id="commandDetail" class="missionReason" style="margin-top:14px">--</div>
        </section>

        <section class="card">
          <details id="rawPanel" class="debugDrawer">
            <summary>Raw Snapshot Console /api/status</summary>
            <pre id="raw">--</pre>
          </details>
        </section>
      </div>
    </main>
  </div>

  <script>
    const maxPoints = 30;
    const refreshPeriodMs = 2000;
    const tracePeriodMs = 5000;
    const rawUpdatePeriodMs = 10000;
    let lastRawUpdateMs = 0;
    let lastTraceDrawMs = 0;
    let lastInstrumentSignature = '';
    let canvasDirty = true;
    let refreshInFlight = false;
    const nodeCache = {};
    const history = { battery: [], altitude: [], yaw: [] };
    const deg = (rad) => Number.isFinite(rad) ? rad * 180 / Math.PI : 0;
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
    const fmt = (v, n = 2) => Number.isFinite(v) ? Number(v).toFixed(n) : '--';

    function node(id) {
      return nodeCache[id] || (nodeCache[id] = document.getElementById(id));
    }
    function statusClass(ok, warn = false) { return ok ? 'ok' : (warn ? 'warn' : 'bad'); }
    function setText(id, text, cls) {
      const el = node(id);
      if (!el) return;
      const nextText = String(text);
      if (el.textContent !== nextText) el.textContent = nextText;
      if (cls) {
        const base = el.dataset.baseClass || (el.dataset.baseClass = el.className.split(' ')[0] || '');
        const nextClass = (base + ' ' + cls).trim();
        if (el.className !== nextClass) el.className = nextClass;
      }
    }
    function setBadge(id, label, cls) {
      const el = node(id);
      if (!el) return;
      const nextClass = 'statusLight ' + cls;
      if (el.className !== nextClass) el.className = nextClass;
      const nextLabel = String(label);
      if (el.dataset.label !== nextLabel) {
        el.dataset.label = nextLabel;
        el.innerHTML = '<span class="dot"></span>' + nextLabel;
      }
    }
    function setGate(id, ok, textOk = 'YES', textBad = 'NO', warn = false) {
      const el = node(id);
      if (!el) return;
      const nextText = ok ? textOk : textBad;
      const nextClass = 'state ' + statusClass(ok, warn && !ok);
      if (el.textContent !== nextText) el.textContent = nextText;
      if (el.className !== nextClass) el.className = nextClass;
    }
    function pushHist(key, value) {
      history[key].push(value);
      if (history[key].length > maxPoints) history[key].shift();
    }
    function resizeCanvas(canvas) {
      const dpr = 1;
      const rect = canvas.getBoundingClientRect();
      const w = Math.max(1, Math.floor(rect.width * dpr));
      const h = Math.max(1, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { ctx, w: rect.width, h: rect.height };
    }

    function drawTrace() {
      const canvas = node('traceCanvas');
      const { ctx, w, h } = resizeCanvas(canvas);
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = 'rgba(99,179,237,.14)';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 6; i++) {
        const y = h * i / 6;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
      }
      drawSeries(ctx, history.battery, w, h, 0, 100, '#50fa7b', 'BAT %', 16);
      let altMax = 3;
      for (const value of history.altitude) altMax = Math.max(altMax, Math.abs(value));
      drawSeries(ctx, history.altitude, w, h, -altMax, altMax, '#37e8ff', 'ALT m', 36);
      drawSeries(ctx, history.yaw, w, h, -180, 180, '#b794f4', 'YAW °', 56);
    }

    function drawSeries(ctx, data, w, h, min, max, color, label, labelY) {
      if (data.length < 2) return;
      const pad = 14;
      const range = max - min || 1;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      data.forEach((v, i) => {
        const x = pad + (i / (maxPoints - 1)) * (w - pad * 2);
        const y = h - pad - ((v - min) / range) * (h - pad * 2);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.font = '11px ui-sans-serif, system-ui';
      ctx.fillText(label + ' ' + fmt(data[data.length - 1], 1), 12, labelY);
    }

    function drawHorizon(rollDeg, pitchDeg) {
      const canvas = node('horizonCanvas');
      const { ctx, w, h } = resizeCanvas(canvas);
      ctx.clearRect(0, 0, w, h);
      ctx.save();
      ctx.translate(w / 2, h / 2);
      ctx.rotate(-rollDeg * Math.PI / 180);
      const pitchPx = clamp(pitchDeg, -30, 30) * 3.2;
      ctx.translate(0, pitchPx);
      ctx.fillStyle = '#15365f';
      ctx.fillRect(-w, -h * 2, w * 2, h * 2);
      ctx.fillStyle = '#3c2f23';
      ctx.fillRect(-w, 0, w * 2, h * 2);
      ctx.strokeStyle = 'rgba(255,255,255,.75)';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(-w, 0); ctx.lineTo(w, 0); ctx.stroke();
      ctx.strokeStyle = 'rgba(255,255,255,.35)';
      for (let p = -30; p <= 30; p += 10) {
        const y = -p * 3.2;
        ctx.beginPath(); ctx.moveTo(-42, y); ctx.lineTo(42, y); ctx.stroke();
      }
      ctx.restore();
      ctx.strokeStyle = '#37e8ff';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(w/2 - 55, h/2); ctx.lineTo(w/2 - 12, h/2); ctx.lineTo(w/2, h/2 + 10); ctx.lineTo(w/2 + 12, h/2); ctx.lineTo(w/2 + 55, h/2);
      ctx.stroke();
      ctx.fillStyle = '#e7f4ff';
      ctx.font = '12px ui-sans-serif, system-ui';
      ctx.fillText('ROLL ' + fmt(rollDeg, 1) + '°', 14, 22);
      ctx.fillText('PITCH ' + fmt(pitchDeg, 1) + '°', 14, 40);
    }

    function drawCompass(yawDeg) {
      const canvas = node('compassCanvas');
      const { ctx, w, h } = resizeCanvas(canvas);
      ctx.clearRect(0, 0, w, h);
      const cx = w / 2, cy = h / 2, r = Math.min(w, h) * .38;
      ctx.strokeStyle = 'rgba(55,232,255,.35)';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
      for (let a = 0; a < 360; a += 15) {
        const ang = (a - 90 - yawDeg) * Math.PI / 180;
        const len = a % 45 === 0 ? 14 : 7;
        ctx.strokeStyle = a % 45 === 0 ? 'rgba(231,244,255,.75)' : 'rgba(231,244,255,.28)';
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(ang) * (r - len), cy + Math.sin(ang) * (r - len));
        ctx.lineTo(cx + Math.cos(ang) * r, cy + Math.sin(ang) * r);
        ctx.stroke();
      }
      const labels = [['N',0],['E',90],['S',180],['W',270]];
      ctx.font = '18px ui-sans-serif, system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (const [lab, a] of labels) {
        const ang = (a - 90 - yawDeg) * Math.PI / 180;
        ctx.fillStyle = lab === 'N' ? '#ff5f75' : '#e7f4ff';
        ctx.fillText(lab, cx + Math.cos(ang) * (r - 28), cy + Math.sin(ang) * (r - 28));
      }
      ctx.fillStyle = '#37e8ff';
      ctx.beginPath();
      ctx.moveTo(cx, cy - r - 12); ctx.lineTo(cx - 8, cy - r + 8); ctx.lineTo(cx + 8, cy - r + 8); ctx.closePath(); ctx.fill();
      ctx.fillStyle = '#e7f4ff'; ctx.font = '34px ui-sans-serif, system-ui';
      ctx.fillText(Math.round(((yawDeg % 360) + 360) % 360) + '°', cx, cy + 4);
    }

    function updateBars(c) {
      const rows = [
        ['Forward', c.forward_m_s, 'barForward', 'valForward', 1.0],
        ['Right', c.right_m_s, 'barRight', 'valRight', 1.0],
        ['Down', c.down_m_s, 'barDown', 'valDown', 1.0],
        ['Yaw', c.yaw_rate_rad_s, 'barYaw', 'valYaw', 0.8],
      ];
      rows.forEach(([_, value, barId, valId, maxAbs]) => {
        const v = Number(value) || 0;
        const pct = 50 + clamp(v / maxAbs, -1, 1) * 50;
        const bar = node(barId);
        const val = node(valId);
        const width = pct.toFixed(1) + '%';
        const text = fmt(v, 3);
        if (bar && bar.style.width !== width) bar.style.width = width;
        if (val && val.textContent !== text) val.textContent = text;
      });
    }

    function updateHud(s) {
      const hud = node('hud');
      const target = s.target || {};
      const errX = Number(target.error_x) || 0;
      const errY = Number(target.error_y) || 0;
      const visible = !!target.target_visible;
      hud.classList.toggle('lost', !visible);
      const offsetX = -clamp(errX, -1, 1) * hud.clientWidth * 0.33;
      const offsetY = clamp(errY, -1, 1) * hud.clientHeight * 0.28;
      const box = node('targetBox');
      const boxTransform = `translate(-50%, -50%) translate(${offsetX.toFixed(1)}px, ${offsetY.toFixed(1)}px)`;
      if (box.style.transform !== boxTransform) box.style.transform = boxTransform;
      setText('targetTag', visible ? (target.tracking_state || 'LOCKED') : 'TARGET LOST');
      const len = Math.min(260, Math.sqrt(offsetX*offsetX + offsetY*offsetY));
      const angle = Math.atan2(offsetY, offsetX) * 180 / Math.PI;
      const vec = node('errorVector');
      const vecWidth = len.toFixed(1) + 'px';
      const vecTransform = `rotate(${angle.toFixed(1)}deg)`;
      if (vec.style.width !== vecWidth) vec.style.width = vecWidth;
      if (vec.style.transform !== vecTransform) vec.style.transform = vecTransform;
      setText('hudClass', target.target_class || '--');
      setText('hudConfidence', fmt(Number(target.confidence), 2));
      setText('hudDetections', String((s.detections || {}).count ?? 0));
      setText('hudDistance', target.distance_valid ? fmt(Number(target.distance_m), 2) + ' m' : '--');
      setText('hudErrX', fmt(errX, 3));
      setText('hudErrY', fmt(errY, 3));
      setText('hudAge', target.age_s == null ? '--' : target.age_s + 's');
      const targetCls = statusClass(visible, true);
      setBadge('targetBadge', visible ? 'LOCKED' : 'SEARCHING', targetCls);
    }

    function updatePreflight(s, t, target) {
      const detections = s.detections || {};
      const telemetryAge = Number(t.age_s);
      const visionAge = Number(detections.age_s);
      const targetAge = Number(target.age_s);
      const battery = Number(t.battery_percent);
      const telemetryFresh = Number.isFinite(telemetryAge) && telemetryAge <= 2.5;
      const visionFresh = Number.isFinite(visionAge) && visionAge <= 2.5;
      const targetFresh = Number.isFinite(targetAge) && targetAge <= 2.5;
      const batteryKnown = Number.isFinite(battery) && battery > 0;
      const batteryOk = batteryKnown && battery >= 25;
      const items = [
        ['preflightTelemetry', telemetryFresh, telemetryFresh ? 'OK' : (Number.isFinite(telemetryAge) ? telemetryAge.toFixed(1) + 's' : 'NO'), false],
        ['preflightLink', !!t.connected, t.connected ? 'YES' : 'NO', false],
        ['preflightArmed', !!t.armed, t.armed ? 'YES' : 'NO', false],
        ['preflightBattery', batteryOk, batteryKnown ? Math.round(battery) + '%' : 'UNKNOWN', true],
        ['preflightLocal', !!t.local_position_valid, t.local_position_valid ? 'VALID' : 'NO', false],
        ['preflightVision', visionFresh, visionFresh ? 'OK' : (Number.isFinite(visionAge) ? visionAge.toFixed(1) + 's' : 'NO'), true],
        ['preflightTarget', !!target.target_visible && targetFresh, target.target_visible ? 'LOCKED' : 'LOST', true],
      ];
      let passed = 0;
      for (const [id, ok, label, warn] of items) {
        if (ok) passed += 1;
        setText(id, label, statusClass(ok, warn && !ok));
      }
      setBadge('preflightSummary', passed + '/' + items.length, passed === items.length ? 'ok' : (passed >= 5 ? 'warn' : 'bad'));
    }

    async function postJson(path, payload) {
      await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload || {}) });
      await refresh();
    }

    async function postBool(path, enabled) { await postJson(path, { enabled }); }

    async function setAutonomy(enabled) { await postBool('/api/autonomy_request', enabled); }
    async function setMavsdk(enabled) { await postBool('/api/mavsdk_request', enabled); }
    async function setMission(enabled) { await postBool('/api/mission_request', enabled); }

    async function systemReady() { await setAutonomy(true); }
    async function startSmartMission() { await setMission(true); }
    async function abortHold() { await postJson('/api/abort_hold', { confirm: true }); }
    async function landNow() { await postJson('/api/land', { confirm: true }); }

    async function refresh() {
      if (document.hidden || refreshInFlight) return;
      refreshInFlight = true;
      try {
        const r = await fetch('/api/status', { cache: 'no-store' });
        const s = await r.json();
        const t = s.telemetry || {};
        const c = s.control || {};
        const target = s.target || {};
        const nowMs = performance.now();
        const mission = (s.mission_state && s.mission_state !== 'UNKNOWN') ? s.mission_state : (s.autonomy_state || 'UNKNOWN');
        const missionParts = String(mission).split(':');
        const missionName = missionParts[0].trim();
        const tracking = missionName.includes('TRACKING') || missionName.includes('READY');
        const waiting = missionName.includes('PREFLIGHT') || missionName.includes('TARGET_LOST') || missionName.includes('REQUESTED');
        setText('mission', missionName, statusClass(tracking, waiting));
        setText('missionReason', missionParts.slice(1).join(':').trim() || s.state_reason || '--');
        setBadge('missionBadge', tracking ? 'LIVE' : waiting ? 'STANDBY' : 'BLOCKED', statusClass(tracking, waiting));

        setText('pillLink', t.connected ? 'CONNECTED' : 'NO LINK', statusClass(t.connected));
        setText('pillArmed', t.armed ? 'TRUE' : 'FALSE', statusClass(t.armed));
        setText('pillMode', t.flight_mode || 'UNKNOWN', statusClass(t.flight_mode && t.flight_mode !== 'UNKNOWN', true));
        setText('pillUptime', fmt(Number(s.uptime_s), 1) + 's', 'cyan');

        const battery = clamp(Number(t.battery_percent) || 0, 0, 100);
        setText('batteryBig', fmt(battery, 0) + '%');
        setText('batteryVolt', fmt(Number(t.battery_voltage), 2) + ' V');

        updateHud(s);

        const rollDeg = t.roll_deg ?? deg(t.roll_rad || 0);
        const pitchDeg = t.pitch_deg ?? deg(t.pitch_rad || 0);
        const yawDeg = t.yaw_deg ?? deg(t.yaw_rad || 0);
        const vn = Number(t.velocity_north_m_s) || 0;
        const ve = Number(t.velocity_east_m_s) || 0;
        const speed = Math.sqrt(vn*vn + ve*ve);
        setText('altitudeMetric', fmt(Number(t.relative_altitude_m), 2) + ' m');
        setText('yawMetric', Math.round(((yawDeg % 360) + 360) % 360) + '°');
        setText('satMetric', String(t.gps_num_satellites ?? 0));
        setText('gpsMetric', 'fix ' + (t.gps_fix_type ?? '--') + ' · health ' + (t.health_gps_ok ? 'OK' : 'NO'));
        setText('velMetric', fmt(speed, 2));
        setText('telemetryAge', t.age_s == null ? '--' : 'age ' + t.age_s + 's');

        setText('attitudeText', `roll ${fmt(rollDeg,1)}° · pitch ${fmt(pitchDeg,1)}°`, 'cyan');
        setText('compassText', `yaw ${fmt(yawDeg,1)}°`, 'cyan');

        updatePreflight(s, t, target);

        pushHist('battery', battery);
        pushHist('altitude', Number(t.relative_altitude_m) || 0);
        pushHist('yaw', yawDeg);
        const instrumentSignature = `${Math.round(rollDeg)}:${Math.round(pitchDeg)}:${Math.round(yawDeg)}`;
        if (canvasDirty || instrumentSignature !== lastInstrumentSignature) {
          drawHorizon(rollDeg, pitchDeg);
          drawCompass(yawDeg);
          lastInstrumentSignature = instrumentSignature;
        }
        if (canvasDirty || nowMs - lastTraceDrawMs > tracePeriodMs) {
          drawTrace();
          lastTraceDrawMs = nowMs;
        }
        canvasDirty = false;

        setGate('gateLink', !!t.connected);
        setGate('gateArmed', !!t.armed);
        setGate('gateOffboard', !!s.offboard_enabled);
        setGate('gateAuto', !!s.autonomy_enabled);
        setGate('gateTarget', !!target.target_visible, 'LOCKED', 'LOST', true);
        setGate('gateGps', !!t.health_gps_ok, 'OK', 'NO', true);
        setGate('gateCommand', !!c.executed, 'SENT', c.command_type || 'IDLE', true);
        setGate('gateMavsdkReq', !!s.mavsdk_requested, 'ON', 'OFF');
        setText('gateMavsdk', s.mavsdk_status || 'UNKNOWN', statusClass(String(s.mavsdk_status).includes('SENT') || String(s.mavsdk_status).includes('OK') || String(s.mavsdk_status).includes('READY'), true));
        const allGo = !!t.connected && !!t.armed && !!s.offboard_enabled && !!s.autonomy_enabled && !!target.target_visible;
        setText('gateSummary', allGo ? 'GREEN' : 'BLOCKED', statusClass(allGo, true));

        setText('commandMain', c.executed ? 'SENT' : c.command_type || 'IDLE', statusClass(c.executed, true));
        setText('commandDetail', `status=${c.status || '--'} · yaw=${fmt(Number(c.yaw_rate_rad_s), 3)} rad/s · fwd=${fmt(Number(c.forward_m_s), 3)} m/s`);
        updateBars(c);

        if (node('rawPanel').open && nowMs - lastRawUpdateMs > rawUpdatePeriodMs) {
          setText('raw', JSON.stringify(s));
          lastRawUpdateMs = nowMs;
        }
      } catch (e) {
        setText('mission', 'DASHBOARD ERROR', 'bad');
        setText('missionReason', String(e));
      } finally {
        refreshInFlight = false;
      }
    }

    window.addEventListener('resize', () => { canvasDirty = true; });
    setInterval(refresh, refreshPeriodMs);
    refresh();
  </script>
</body>
</html>"""

class DashboardHandler(BaseHTTPRequestHandler):
    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - http.server API name
        if self.path == "/" or self.path.startswith("/?"):
            self._send(200, DASHBOARD_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path == "/api/status":
            snapshot_fn: Callable[[], dict[str, Any]] = self.server.snapshot_fn  # type: ignore[attr-defined]
            body = json.dumps(snapshot_fn()).encode("utf-8")
            self._send(200, body, "application/json")
            return
        self._send(404, b"not found", "text/plain")

    @staticmethod
    def _payload_bool(payload: dict[str, Any], field: str, default: bool = False) -> bool:
        value = payload.get(field, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def do_POST(self) -> None:  # noqa: N802 - http.server API name
        request_spec = {
            "/api/autonomy_request": ("request_autonomy_fn", "enabled"),
            "/api/mavsdk_request": ("request_mavsdk_fn", "enabled"),
            "/api/mission_request": ("request_mission_fn", "enabled"),
            "/api/abort_hold": ("request_abort_hold_fn", "confirm"),
            "/api/land": ("request_land_fn", "confirm"),
        }.get(self.path)
        if request_spec is None:
            self._send(404, b"not found", "text/plain")
            return

        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(min(length, 1024)) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
        except Exception:
            self._send(400, b"invalid json", "text/plain")
            return

        request_attr, bool_field = request_spec
        value = self._payload_bool(payload, bool_field, False)

        if bool_field == "confirm" and not value:
            body = json.dumps({"ok": False, "error": "confirm true required"}).encode("utf-8")
            self._send(400, body, "application/json")
            return

        request_fn: Callable[[bool], None] = getattr(self.server, request_attr)  # type: ignore[attr-defined]
        request_fn(value)

        response_field = "confirmed" if bool_field == "confirm" else "enabled"
        self._send(200, json.dumps({"ok": True, response_field: value}).encode("utf-8"), "application/json")

    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep normal HTTP polling from spamming ROS logs.
        return


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/drone/vision/detections")
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("control_command_topic", "/drone/control/command")
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("mavsdk_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("mission_request_topic", "/drone/mission/request")
        self.declare_parameter("autonomy_enable_topic", "/drone/autonomy/enabled")
        self.declare_parameter("offboard_enable_topic", "/drone/mavsdk/offboard_enable")
        self.declare_parameter("autonomy_state_topic", "/drone/autonomy/state")
        self.declare_parameter("mission_state_topic", "/drone/mission/state")
        self.declare_parameter("mavsdk_command_status_topic", "/drone/mavsdk/command_status")
        self.declare_parameter("mavsdk_action_topic", "/drone/mavsdk/action_command")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.control_command_topic = str(self.get_parameter("control_command_topic").value)
        self.autonomy_request_topic = str(self.get_parameter("autonomy_request_topic").value)
        self.mavsdk_request_topic = str(self.get_parameter("mavsdk_request_topic").value)
        self.mission_request_topic = str(self.get_parameter("mission_request_topic").value)
        self.autonomy_enable_topic = str(self.get_parameter("autonomy_enable_topic").value)
        self.offboard_enable_topic = str(self.get_parameter("offboard_enable_topic").value)
        self.autonomy_state_topic = str(self.get_parameter("autonomy_state_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.mavsdk_command_status_topic = str(self.get_parameter("mavsdk_command_status_topic").value)
        self.mavsdk_action_topic = str(self.get_parameter("mavsdk_action_topic").value)

        self._lock = threading.Lock()
        self._started_at = time.time()
        self._last_autonomy_request: bool | None = None
        self._last_mavsdk_request: bool | None = None
        self._last_mission_request: bool | None = None
        self._last_action_request: str | None = None
        self._action_command_id: int = 0
        self._snapshot: dict[str, Any] = self._empty_snapshot()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.autonomy_request_pub = self.create_publisher(Bool, self.autonomy_request_topic, qos)
        self.mavsdk_request_pub = self.create_publisher(Bool, self.mavsdk_request_topic, qos)
        self.mission_request_pub = self.create_publisher(Bool, self.mission_request_topic, qos)
        self.action_pub = self.create_publisher(MavsdkActionCommand, self.mavsdk_action_topic, qos)
        self.create_subscription(DetectionArray, self.detections_topic, self._detections_cb, qos)
        self.create_subscription(TargetError, self.target_error_topic, self._target_cb, qos)
        self.create_subscription(DroneTelemetry, self.telemetry_topic, self._telemetry_cb, qos)
        self.create_subscription(ControlCommand, self.control_command_topic, self._control_cb, qos)
        self.create_subscription(Bool, self.autonomy_enable_topic, self._autonomy_enabled_cb, qos)
        self.create_subscription(Bool, self.mavsdk_request_topic, self._mavsdk_requested_cb, qos)
        self.create_subscription(Bool, self.offboard_enable_topic, self._offboard_enabled_cb, qos)
        self.create_subscription(String, self.autonomy_state_topic, self._autonomy_state_cb, qos)
        self.create_subscription(String, self.mission_state_topic, self._mission_state_cb, qos)
        self.create_subscription(String, self.mavsdk_command_status_topic, self._mavsdk_status_cb, qos)

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.detections_topic, "detections")
        self.diagnostics.add_input(self.target_error_topic, "target_error")
        self.diagnostics.add_input(self.telemetry_topic, "telemetry")
        self.diagnostics.add_input(self.control_command_topic, "control_command")
        self.diagnostics.add_input(self.autonomy_state_topic, "autonomy_state")
        self.diagnostics.add_input(self.mission_state_topic, "mission_state")
        self.diagnostics.add_input(self.mavsdk_request_topic, "mavsdk_request", stale_seconds=60.0)
        self.diagnostics.add_output(self.autonomy_request_topic, "autonomy_request")
        self.diagnostics.add_output(self.mavsdk_request_topic, "mavsdk_request")
        self.diagnostics.add_output(self.mission_request_topic, "mission_request")
        self.diagnostics.add_output(self.mavsdk_action_topic, "mavsdk_action_command")

        self._server = ThreadingHTTPServer((self.host, self.port), DashboardHandler)
        self._server.snapshot_fn = self.get_snapshot  # type: ignore[attr-defined]
        self._server.request_autonomy_fn = self.publish_autonomy_request  # type: ignore[attr-defined]
        self._server.request_mavsdk_fn = self.publish_mavsdk_request  # type: ignore[attr-defined]
        self._server.request_mission_fn = self.publish_mission_request  # type: ignore[attr-defined]
        self._server.request_abort_hold_fn = self.publish_abort_hold  # type: ignore[attr-defined]
        self._server.request_land_fn = self.publish_land  # type: ignore[attr-defined]
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self.get_logger().warning(
            f"Dashboard available at http://{self.host}:{self.port}/ | "
            f"mission_state={self.mission_state_topic}, autonomy_request={self.autonomy_request_topic}, "
            f"mavsdk_request={self.mavsdk_request_topic}, mission_request={self.mission_request_topic}, "
            f"mavsdk_action={self.mavsdk_action_topic}"
        )

    def _empty_snapshot(self) -> dict[str, Any]:
        return {
            "uptime_s": 0.0,
            "autonomy_state": "UNKNOWN",
            "mission_state": "UNKNOWN",
            "state_reason": "startup",
            "autonomy_enabled": False,
            "mavsdk_requested": False,
            "offboard_enabled": False,
            "last_autonomy_request": None,
            "last_mavsdk_request": None,
            "last_mission_request": None,
            "last_action_request": None,
            "mavsdk_status": "UNKNOWN",
            "detections": {"count": 0, "age_s": None},
            "target": {
                "tracking_state": "UNKNOWN",
                "target_visible": False,
                "target_class": "",
                "confidence": 0.0,
                "area": 0.0,
                "distance_valid": False,
                "distance_m": 0.0,
                "raw_distance_m": 0.0,
                "bearing_x_rad": 0.0,
                "bearing_y_rad": 0.0,
                "target_diameter_px": 0.0,
                "error_x": 0.0,
                "error_y": 0.0,
                "age_s": None,
            },
            "telemetry": {
                "connected": False,
                "armed": False,
                "flight_mode": "UNKNOWN",
                "landed_state": "UNKNOWN",
                "battery_percent": 0.0,
                "battery_voltage": 0.0,
                "relative_altitude_m": 0.0,
                "absolute_altitude_m": 0.0,
                "roll_rad": 0.0,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "roll_deg": 0.0,
                "pitch_deg": 0.0,
                "yaw_deg": 0.0,
                "velocity_north_m_s": 0.0,
                "velocity_east_m_s": 0.0,
                "velocity_down_m_s": 0.0,
                "gps_num_satellites": 0,
                "gps_fix_type": 0,
                "health_all_ok": False,
                "health_gps_ok": False,
                "age_s": None,
            },
            "control": {
                "command_type": "UNKNOWN",
                "executed": False,
                "status": "UNKNOWN",
                "forward_m_s": 0.0,
                "right_m_s": 0.0,
                "down_m_s": 0.0,
                "yaw_rate_rad_s": 0.0,
                "age_s": None,
            },
            "_timestamps": {},
        }

    def get_snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            snapshot = copy.deepcopy(self._snapshot)

        snapshot["uptime_s"] = round(now - self._started_at, 1)
        timestamps = snapshot.pop("_timestamps", {})
        for section, timestamp in timestamps.items():
            if timestamp:
                if section in snapshot and isinstance(snapshot[section], dict):
                    snapshot[section]["age_s"] = round(now - float(timestamp), 2)
                else:
                    snapshot[f"{section}_age_s"] = round(now - float(timestamp), 2)
        return snapshot

    def _set_section(self, section: str, data: dict[str, Any]) -> None:
        now = time.time()
        with self._lock:
            self._snapshot[section].update(data)
            self._snapshot["_timestamps"][section] = now

    def _detections_cb(self, msg: DetectionArray) -> None:
        count = int(msg.count) if msg.count >= 0 else len(msg.detections)
        self._set_section("detections", {"count": count})
        self.diagnostics.mark_received(self.detections_topic, summary=f"count={count}")

    def _target_cb(self, msg: TargetError) -> None:
        self._set_section(
            "target",
            {
                "tracking_state": msg.tracking_state,
                "target_visible": bool(msg.target_visible),
                "target_class": msg.target_class,
                "confidence": round(float(msg.target_confidence), 3),
                "area": round(float(msg.target_area), 4),
                "distance_valid": bool(getattr(msg, "distance_valid", False)),
                "distance_m": round(float(getattr(msg, "distance_m", 0.0)), 3),
                "raw_distance_m": round(float(getattr(msg, "raw_distance_m", 0.0)), 3),
                "bearing_x_rad": round(float(getattr(msg, "bearing_x_rad", 0.0)), 4),
                "bearing_y_rad": round(float(getattr(msg, "bearing_y_rad", 0.0)), 4),
                "target_diameter_px": round(float(getattr(msg, "target_diameter_px", 0.0)), 1),
                "error_x": round(float(msg.error_x), 3),
                "error_y": round(float(msg.error_y), 3),
            },
        )
        self.diagnostics.mark_received(self.target_error_topic, summary=f"state={msg.tracking_state}, visible={msg.target_visible}, distance={getattr(msg, 'distance_m', 0.0):.2f}")

    def _telemetry_cb(self, msg: DroneTelemetry) -> None:
        self._set_section(
            "telemetry",
            {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "flight_mode": msg.flight_mode,
                "landed_state": msg.landed_state,
                "battery_percent": round(float(msg.battery_remaining_percent), 1),
                "battery_voltage": round(float(msg.battery_voltage), 2),
                "relative_altitude_m": round(float(msg.relative_altitude), 2),
                "absolute_altitude_m": round(float(msg.absolute_altitude), 2),
                "local_position_valid": bool(getattr(msg, "local_position_valid", False)),
                "local_position_north_m": round(float(getattr(msg, "local_position_north", 0.0)), 2),
                "local_position_east_m": round(float(getattr(msg, "local_position_east", 0.0)), 2),
                "local_position_down_m": round(float(getattr(msg, "local_position_down", 0.0)), 2),
                "roll_rad": round(float(msg.roll), 4),
                "pitch_rad": round(float(msg.pitch), 4),
                "yaw_rad": round(float(msg.yaw), 4),
                "roll_deg": round(float(msg.roll) * 57.2957795131, 1),
                "pitch_deg": round(float(msg.pitch) * 57.2957795131, 1),
                "yaw_deg": round(float(msg.yaw) * 57.2957795131, 1),
                "velocity_north_m_s": round(float(msg.velocity_north), 3),
                "velocity_east_m_s": round(float(msg.velocity_east), 3),
                "velocity_down_m_s": round(float(msg.velocity_down), 3),
                "gps_num_satellites": int(msg.gps_num_satellites),
                "gps_fix_type": int(msg.gps_fix_type),
                "health_all_ok": bool(msg.health_all_ok),
                "health_gps_ok": bool(msg.health_gps_ok),
            },
        )
        self.diagnostics.mark_received(self.telemetry_topic, summary=f"connected={msg.connected}, armed={msg.armed}")

    def _control_cb(self, msg: ControlCommand) -> None:
        self._set_section(
            "control",
            {
                "command_type": msg.command_type,
                "executed": bool(msg.executed),
                "status": msg.execution_status,
                "forward_m_s": round(float(msg.velocity_forward), 3),
                "right_m_s": round(float(msg.velocity_right), 3),
                "down_m_s": round(float(msg.velocity_down), 3),
                "yaw_rate_rad_s": round(float(msg.yaw_rate), 3),
                "position_valid": bool(getattr(msg, "position_valid", False)),
                "position_north_m": round(float(getattr(msg, "position_north", 0.0)), 2),
                "position_east_m": round(float(getattr(msg, "position_east", 0.0)), 2),
                "position_down_m": round(float(getattr(msg, "position_down", 0.0)), 2),
                "yaw_deg": round(float(getattr(msg, "yaw_deg", 0.0)), 1),
            },
        )
        self.diagnostics.mark_received(self.control_command_topic, summary=f"status={msg.execution_status}, executed={msg.executed}")

    def _autonomy_enabled_cb(self, msg: Bool) -> None:
        with self._lock:
            self._snapshot["autonomy_enabled"] = bool(msg.data)
            self._snapshot["_timestamps"]["autonomy_enabled"] = time.time()

    def _mavsdk_requested_cb(self, msg: Bool) -> None:
        with self._lock:
            self._snapshot["mavsdk_requested"] = bool(msg.data)
            self._snapshot["_timestamps"]["mavsdk_requested"] = time.time()
        self.diagnostics.mark_received(self.mavsdk_request_topic, summary=f"requested={msg.data}")

    def _offboard_enabled_cb(self, msg: Bool) -> None:
        with self._lock:
            self._snapshot["offboard_enabled"] = bool(msg.data)
            self._snapshot["_timestamps"]["offboard_enabled"] = time.time()

    def _autonomy_state_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["autonomy_state"] = msg.data
            self._snapshot["state_reason"] = msg.data.split(":", 1)[1].strip() if ":" in msg.data else ""
            self._snapshot["_timestamps"]["autonomy_state"] = time.time()
        self.diagnostics.mark_received(self.autonomy_state_topic, summary=msg.data)

    def _mission_state_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["mission_state"] = msg.data
            self._snapshot["state_reason"] = msg.data.split(":", 1)[1].strip() if ":" in msg.data else ""
            self._snapshot["_timestamps"]["mission_state"] = time.time()
        self.diagnostics.mark_received(self.mission_state_topic, summary=msg.data)

    def _mavsdk_status_cb(self, msg: String) -> None:
        with self._lock:
            self._snapshot["mavsdk_status"] = msg.data
            self._snapshot["_timestamps"]["mavsdk_status"] = time.time()

    def publish_autonomy_request(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.autonomy_request_pub.publish(msg)
        with self._lock:
            self._last_autonomy_request = bool(enabled)
            self._snapshot["last_autonomy_request"] = bool(enabled)
            self._snapshot["_timestamps"]["last_autonomy_request"] = time.time()
        self.diagnostics.mark_published(self.autonomy_request_topic, summary=f"requested={enabled}")
        self.get_logger().warning(f"Dashboard autonomy request: {enabled}")

    def publish_mavsdk_request(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.mavsdk_request_pub.publish(msg)
        with self._lock:
            self._last_mavsdk_request = bool(enabled)
            self._snapshot["last_mavsdk_request"] = bool(enabled)
            self._snapshot["mavsdk_requested"] = bool(enabled)
            self._snapshot["_timestamps"]["last_mavsdk_request"] = time.time()
            self._snapshot["_timestamps"]["mavsdk_requested"] = time.time()
        self.diagnostics.mark_published(self.mavsdk_request_topic, summary=f"requested={enabled}")
        self.get_logger().warning(f"Dashboard MAVSDK offboard request: {enabled}")

    def publish_mission_request(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.mission_request_pub.publish(msg)
        with self._lock:
            self._last_mission_request = bool(enabled)
            self._snapshot["last_mission_request"] = bool(enabled)
            self._snapshot["_timestamps"]["last_mission_request"] = time.time()
        self.diagnostics.mark_published(self.mission_request_topic, summary=f"requested={enabled}")
        self.get_logger().warning(f"Dashboard mission request: {enabled}")

    def _publish_action_request(self, action: str, note: str) -> None:
        self._action_command_id += 1
        msg = MavsdkActionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = int(self._action_command_id)
        msg.action = action
        msg.execute = True
        msg.takeoff_altitude_m = 0.0
        msg.radius_m = 0.0
        msg.velocity_m_s = 0.0
        msg.orbit_revolutions = 0.0
        msg.yaw_behavior = ""
        msg.latitude_deg = 0.0
        msg.longitude_deg = 0.0
        msg.absolute_altitude_m = 0.0
        msg.note = note
        self.action_pub.publish(msg)
        with self._lock:
            self._last_action_request = action
            self._snapshot["last_action_request"] = action
            self._snapshot["_timestamps"]["last_action_request"] = time.time()
        self.diagnostics.mark_published(self.mavsdk_action_topic, summary=f"id={msg.command_id}, action={action}, note={note}")
        self.get_logger().warning(f"Dashboard MAVSDK action request: {action} | {note}")

    def publish_abort_hold(self, confirmed: bool = False) -> None:
        if not confirmed:
            self.get_logger().warning("Dashboard Abort/Hold ignored because confirm=true was not provided.")
            return

        # Abort should stop our mission/offboard requests first, then ask PX4 to hold if actions are enabled.
        self.publish_mission_request(False)
        self.publish_mavsdk_request(False)
        self.publish_autonomy_request(False)
        self._publish_action_request("HOLD", "dashboard Abort/Hold")

    def publish_land(self, confirmed: bool = False) -> None:
        if not confirmed:
            self.get_logger().warning("Dashboard Land ignored because confirm=true was not provided.")
            return

        # Land is intentionally separate from Abort/Hold. telemetry_node still enforces allow_mavsdk_actions.
        self.publish_mission_request(False)
        self.publish_mavsdk_request(False)
        self.publish_autonomy_request(False)
        self._publish_action_request("LAND", "dashboard Land")

    def destroy_node(self) -> None:
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()
        self.get_logger().info("Dashboard node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("dashboard_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
