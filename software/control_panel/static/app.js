const levelPill = document.querySelector("#levelPill");
const cameraStream = document.querySelector("#cameraStream");
const mapCanvas = document.querySelector("#mapCanvas");
const mapOverlay = document.querySelector("#mapOverlay");
const mapHint = document.querySelector("#mapHint");
const mapContext = mapCanvas.getContext("2d");
const fitMapButton = document.querySelector("#fitMapButton");

const fields = {
  mission_status: document.querySelector("#missionStatus"),
  mission_mode_status: document.querySelector("#modeStatus"),
  mission_task_status: document.querySelector("#taskStatus"),
  hazard: document.querySelector("#hazardStatus"),
  battery_status: document.querySelector("#batteryStatus"),
  camera_status: document.querySelector("#cameraStatus"),
  assistance_request: document.querySelector("#assistStatus"),
  elevator_status: document.querySelector("#elevatorStatus"),
  pose_status: document.querySelector("#poseStatus"),
  goal_status: document.querySelector("#goalStatus"),
};
const lastAction = document.querySelector("#lastAction");
const assistMessage = document.querySelector("#assistMessage");
const elevatorMessage = document.querySelector("#elevatorMessage");

let cameraUrlSet = false;
let mapInfo = null;
let mapImage = null;
let robotPose = null;
let goalPose = null;
let mapMode = "goal";
let dragStart = null;
let panStart = null;
let previewTarget = null;
let viewScale = 1;
let viewOffset = { x: 0, y: 0 };
let mapViewInitialized = false;

function setText(node, value) {
  node.textContent = value || "unknown";
}

function setLevel(level) {
  const clean = (level || "UNKNOWN").toUpperCase();
  levelPill.textContent = clean;
  levelPill.className = "pill";
  if (clean === "OK") levelPill.classList.add("ok");
  if (clean === "WARN") levelPill.classList.add("warn");
  if (clean === "DANGER") levelPill.classList.add("danger");
}

function formatPose(pose) {
  if (!pose) return "none";
  const deg = (pose.yaw * 180) / Math.PI;
  return `x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, yaw=${deg.toFixed(0)}deg`;
}

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    const data = await response.json();
    setLevel(data.mission_level);
    setText(fields.mission_status, data.mission_status);
    setText(fields.mission_mode_status, data.mission_mode_status);
    setText(fields.mission_task_status, data.mission_task_status);
    setText(fields.hazard, data.hazard);
    setText(fields.battery_status, data.battery_status);
    setText(fields.camera_status, data.camera_status);
    setText(fields.assistance_request, data.assistance_request);
    setText(fields.elevator_status, data.elevator_status);
    assistMessage.textContent = data.assistance_request || "no request";
    elevatorMessage.textContent = data.elevator_status || "phase=idle";

    robotPose = data.robot_pose || robotPose;
    goalPose = data.goal_pose || goalPose;
    setText(fields.pose_status, formatPose(robotPose));
    setText(fields.goal_status, formatPose(goalPose));
    drawMap();

    if (!cameraUrlSet && data.camera_stream_url) {
      cameraStream.src = data.camera_stream_url;
      cameraUrlSet = true;
    }
  } catch (error) {
    setLevel("UNKNOWN");
    lastAction.textContent = `status error: ${error.message}`;
  }
}

async function loadMap() {
  try {
    const response = await fetch("/api/map", { cache: "no-store" });
    const data = await response.json();
    if (!data.ok) throw new Error(data.error || "map load failed");

    const bytes = Uint8Array.from(atob(data.pixels_b64), (char) =>
      char.charCodeAt(0)
    );
    const imageData = new ImageData(data.width, data.height);
    for (let i = 0; i < bytes.length; i += 1) {
      const value = bytes[i];
      const offset = i * 4;
      imageData.data[offset] = value;
      imageData.data[offset + 1] = value;
      imageData.data[offset + 2] = value;
      imageData.data[offset + 3] = 255;
    }

    const offscreen = document.createElement("canvas");
    offscreen.width = data.width;
    offscreen.height = data.height;
    offscreen.getContext("2d").putImageData(imageData, 0, 0);
    mapImage = offscreen;
    mapInfo = data;
    mapOverlay.textContent = `${data.width}x${data.height}, ${data.resolution}m/px`;
    fitMapToCanvas();
    resizeMapCanvas();
  } catch (error) {
    mapOverlay.textContent = `map error: ${error.message}`;
  }
}

function resizeMapCanvas() {
  const rect = mapCanvas.getBoundingClientRect();
  const nextWidth = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
  const nextHeight = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
  if (mapCanvas.width !== nextWidth || mapCanvas.height !== nextHeight) {
    mapCanvas.width = nextWidth;
    mapCanvas.height = nextHeight;
    if (!mapViewInitialized) fitMapToCanvas();
  }
  drawMap();
}

function fitMapToCanvas() {
  if (!mapInfo) return { scale: 1, x: 0, y: 0 };
  viewScale = Math.min(
    mapCanvas.width / mapInfo.width,
    mapCanvas.height / mapInfo.height
  );
  viewOffset = {
    x: (mapCanvas.width - mapInfo.width * viewScale) / 2,
    y: (mapCanvas.height - mapInfo.height * viewScale) / 2,
  };
  mapViewInitialized = true;
  drawMap();
}

function mapFit() {
  return { scale: viewScale, x: viewOffset.x, y: viewOffset.y };
}

function worldToPixel(x, y) {
  const px = (x - mapInfo.origin[0]) / mapInfo.resolution;
  const py = mapInfo.height - 1 - (y - mapInfo.origin[1]) / mapInfo.resolution;
  return { x: px, y: py };
}

function pixelToWorld(px, py) {
  return {
    x: mapInfo.origin[0] + px * mapInfo.resolution,
    y: mapInfo.origin[1] + (mapInfo.height - 1 - py) * mapInfo.resolution,
  };
}

function canvasToMapPixel(event) {
  if (!mapInfo) return null;
  const rect = mapCanvas.getBoundingClientRect();
  const fit = mapFit();
  const cx = (event.clientX - rect.left) * window.devicePixelRatio;
  const cy = (event.clientY - rect.top) * window.devicePixelRatio;
  const px = (cx - fit.x) / fit.scale;
  const py = (cy - fit.y) / fit.scale;
  if (px < 0 || py < 0 || px >= mapInfo.width || py >= mapInfo.height) {
    return null;
  }
  return { px, py };
}

function canvasPoint(event) {
  const rect = mapCanvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * window.devicePixelRatio,
    y: (event.clientY - rect.top) * window.devicePixelRatio,
  };
}

function drawArrow(ctx, px, py, yaw, color, label) {
  const fit = mapFit();
  const x = fit.x + px * fit.scale;
  const y = fit.y + py * fit.scale;
  const size = Math.max(12, 18 * window.devicePixelRatio);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.fillStyle = color;
  ctx.strokeStyle = "#050806";
  ctx.lineWidth = 2 * window.devicePixelRatio;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.55, size * 0.55);
  ctx.lineTo(-size * 0.25, 0);
  ctx.lineTo(-size * 0.55, -size * 0.55);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  if (label) {
    ctx.fillStyle = color;
    ctx.font = `${12 * window.devicePixelRatio}px Segoe UI`;
    ctx.fillText(label, x + 10, y - 10);
  }
}

function drawMap() {
  mapContext.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  if (!mapInfo || !mapImage) return;

  const fit = mapFit();
  mapContext.imageSmoothingEnabled = false;
  mapContext.drawImage(
    mapImage,
    fit.x,
    fit.y,
    mapInfo.width * fit.scale,
    mapInfo.height * fit.scale
  );

  if (goalPose) {
    const goal = worldToPixel(goalPose.x, goalPose.y);
    drawArrow(mapContext, goal.x, goal.y, goalPose.yaw || 0, "#ffd166", "goal");
  }

  if (robotPose) {
    const robot = worldToPixel(robotPose.x, robotPose.y);
    drawArrow(mapContext, robot.x, robot.y, robotPose.yaw || 0, "#7bd88f", "robot");
  }

  if (previewTarget) {
    const point = worldToPixel(previewTarget.x, previewTarget.y);
    drawArrow(
      mapContext,
      point.x,
      point.y,
      previewTarget.yaw || 0,
      mapMode === "initialpose" ? "#8fd6c9" : "#ffd166",
      mapMode
    );
  }
}

async function sendMapTarget(target) {
  const endpoint = mapMode === "initialpose" ? "/api/initialpose" : "/api/goal";
  const label = mapMode === "initialpose" ? "estimate" : "goal";
  lastAction.textContent = `sending ${label}`;
  try {
    const response = await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(target),
    });
    const data = await response.json();
    if (!data.ok) throw new Error(data.error || `${label} failed`);
    lastAction.textContent = `sent ${label}: ${formatPose(target)}`;
    if (mapMode === "goal") goalPose = target;
    if (mapMode === "initialpose") robotPose = target;
    previewTarget = null;
    drawMap();
  } catch (error) {
    lastAction.textContent = `${label} error: ${error.message}`;
  }
}

function targetFromDrag(start, end) {
  const startWorld = pixelToWorld(start.px, start.py);
  const endWorld = pixelToWorld(end.px, end.py);
  const dx = endWorld.x - startWorld.x;
  const dy = endWorld.y - startWorld.y;
  const yaw = Math.hypot(dx, dy) > 0.05 ? Math.atan2(dy, dx) : 0.0;
  return { x: startWorld.x, y: startWorld.y, yaw };
}

async function sendCommand(command) {
  lastAction.textContent = `sending ${command}`;
  try {
    const response = await fetch("/api/mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command }),
    });
    const data = await response.json();
    if (!data.ok) throw new Error(data.error || "command failed");
    lastAction.textContent = `sent ${command}`;
    await refreshStatus();
  } catch (error) {
    lastAction.textContent = `command error: ${error.message}`;
  }
}

async function sendDecision(decision) {
  lastAction.textContent = `sending decision ${decision}`;
  try {
    const response = await fetch("/api/decision", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ decision }),
    });
    const data = await response.json();
    if (!data.ok) throw new Error(data.error || "decision failed");
    lastAction.textContent = `sent decision ${decision}`;
    await refreshStatus();
  } catch (error) {
    lastAction.textContent = `decision error: ${error.message}`;
  }
}

async function sendElevatorDecision(decision) {
  lastAction.textContent = `sending elevator ${decision}`;
  try {
    const response = await fetch("/api/elevator", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ decision }),
    });
    const data = await response.json();
    if (!data.ok) throw new Error(data.error || "elevator decision failed");
    lastAction.textContent = `sent elevator ${decision}`;
    await refreshStatus();
  } catch (error) {
    lastAction.textContent = `elevator error: ${error.message}`;
  }
}

document.querySelectorAll("button[data-command]").forEach((button) => {
  button.addEventListener("click", () => sendCommand(button.dataset.command));
});

document.querySelectorAll("button[data-decision]").forEach((button) => {
  button.addEventListener("click", () => sendDecision(button.dataset.decision));
});

document.querySelectorAll("button[data-elevator]").forEach((button) => {
  button.addEventListener("click", () =>
    sendElevatorDecision(button.dataset.elevator)
  );
});

document.querySelectorAll("button[data-tab]").forEach((button) => {
  button.addEventListener("click", () => {
    const target = button.dataset.tab;
    document
      .querySelectorAll("button[data-tab]")
      .forEach((item) => item.classList.toggle("active", item === button));
    document
      .querySelectorAll("[data-panel]")
      .forEach((panel) =>
        panel.classList.toggle("active", panel.dataset.panel === target)
      );
  });
});

document.querySelectorAll("button[data-map-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    mapMode = button.dataset.mapMode;
    document
      .querySelectorAll("button[data-map-mode]")
      .forEach((item) => item.classList.toggle("active", item === button));
    const hints = {
      goal: "Goal: click or drag on map",
      initialpose: "Estimate: click or drag robot pose",
      move: "Move: drag map, wheel zoom",
      inspect: "Inspect: map only",
    };
    mapHint.textContent = hints[mapMode] || "";
  });
});

fitMapButton.addEventListener("click", fitMapToCanvas);

document.querySelectorAll("button[data-fullscreen]").forEach((button) => {
  button.addEventListener("click", () => {
    const target = document.querySelector(`#${button.dataset.fullscreen}`);
    if (!target) return;
    if (document.fullscreenElement) {
      document.exitFullscreen();
    } else {
      target.requestFullscreen();
    }
  });
});

mapCanvas.addEventListener("pointerdown", (event) => {
  if (mapMode === "inspect") return;
  if (mapMode === "move") {
    panStart = { point: canvasPoint(event), offset: { ...viewOffset } };
    mapCanvas.setPointerCapture(event.pointerId);
    return;
  }
  dragStart = canvasToMapPixel(event);
  if (dragStart) {
    mapCanvas.setPointerCapture(event.pointerId);
  }
});

mapCanvas.addEventListener("pointermove", (event) => {
  if (panStart && mapMode === "move") {
    const point = canvasPoint(event);
    viewOffset = {
      x: panStart.offset.x + point.x - panStart.point.x,
      y: panStart.offset.y + point.y - panStart.point.y,
    };
    drawMap();
    return;
  }
  if (!dragStart || mapMode === "inspect") return;
  const current = canvasToMapPixel(event);
  if (!current) return;
  previewTarget = targetFromDrag(dragStart, current);
  drawMap();
});

mapCanvas.addEventListener("pointerup", async (event) => {
  if (panStart && mapMode === "move") {
    panStart = null;
    return;
  }
  if (!dragStart || mapMode === "inspect") return;
  const end = canvasToMapPixel(event) || dragStart;
  const target = targetFromDrag(dragStart, end);
  dragStart = null;
  previewTarget = null;
  await sendMapTarget(target);
});

mapCanvas.addEventListener("wheel", (event) => {
  if (!mapInfo) return;
  event.preventDefault();
  const point = canvasPoint(event);
  const before = {
    px: (point.x - viewOffset.x) / viewScale,
    py: (point.y - viewOffset.y) / viewScale,
  };
  const zoom = event.deltaY < 0 ? 1.15 : 1 / 1.15;
  viewScale = Math.max(0.05, Math.min(20, viewScale * zoom));
  viewOffset = {
    x: point.x - before.px * viewScale,
    y: point.y - before.py * viewScale,
  };
  drawMap();
});

window.addEventListener("resize", resizeMapCanvas);
document.addEventListener("fullscreenchange", () => {
  setTimeout(resizeMapCanvas, 50);
});

loadMap();
refreshStatus();
setInterval(refreshStatus, 1500);
