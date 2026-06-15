const levelPill = document.querySelector("#levelPill");
const cameraStream = document.querySelector("#cameraStream");
const fields = {
  mission_status: document.querySelector("#missionStatus"),
  mission_mode_status: document.querySelector("#modeStatus"),
  mission_task_status: document.querySelector("#taskStatus"),
  hazard: document.querySelector("#hazardStatus"),
  camera_status: document.querySelector("#cameraStatus"),
  assistance_request: document.querySelector("#assistStatus"),
};
const lastAction = document.querySelector("#lastAction");
const assistMessage = document.querySelector("#assistMessage");

let cameraUrlSet = false;

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

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    const data = await response.json();
    setLevel(data.mission_level);
    setText(fields.mission_status, data.mission_status);
    setText(fields.mission_mode_status, data.mission_mode_status);
    setText(fields.mission_task_status, data.mission_task_status);
    setText(fields.hazard, data.hazard);
    setText(fields.camera_status, data.camera_status);
    setText(fields.assistance_request, data.assistance_request);
    assistMessage.textContent = data.assistance_request || "no request";
    if (!cameraUrlSet && data.camera_stream_url) {
      cameraStream.src = data.camera_stream_url;
      cameraUrlSet = true;
    }
  } catch (error) {
    setLevel("UNKNOWN");
    lastAction.textContent = `status error: ${error.message}`;
  }
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

document.querySelectorAll("button[data-command]").forEach((button) => {
  button.addEventListener("click", () => sendCommand(button.dataset.command));
});

document.querySelectorAll("button[data-decision]").forEach((button) => {
  button.addEventListener("click", () => sendDecision(button.dataset.decision));
});

refreshStatus();
setInterval(refreshStatus, 1500);
