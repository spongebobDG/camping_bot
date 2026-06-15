#!/usr/bin/env python3
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
DEFAULT_PORT = int(os.environ.get("CAMPING_PANEL_PORT", "8088"))
CAMERA_STREAM_URL = os.environ.get(
    "CAMPING_CAMERA_STREAM_URL", "http://192.168.0.11/stream"
)


class TopicCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "mission_status": "unknown",
            "mission_level": "UNKNOWN",
            "mission_mode_status": "unknown",
            "mission_task_status": "unknown",
            "camera_status": "unknown",
            "camera_online": "unknown",
            "hazard": "unknown",
            "assistance_request": "none",
            "updated_at": 0.0,
        }

    def set(self, key, value):
        with self.lock:
            self.data[key] = value
            self.data["updated_at"] = time.time()

    def snapshot(self):
        with self.lock:
            result = dict(self.data)
        result["camera_stream_url"] = CAMERA_STREAM_URL
        result["panel_time"] = time.time()
        return result


CACHE = TopicCache()


TOPICS = {
    "mission_status": ("/mission/status", "std_msgs/msg/String"),
    "mission_level": ("/mission/level", "std_msgs/msg/String"),
    "mission_mode_status": ("/mission/mode_status", "std_msgs/msg/String"),
    "mission_task_status": ("/mission/task_status", "std_msgs/msg/String"),
    "camera_status": ("/camera/status", "std_msgs/msg/String"),
    "camera_online": ("/camera/online", "std_msgs/msg/Bool"),
    "hazard": ("/camping_robot/hazard", "std_msgs/msg/String"),
    "assistance_request": ("/mission/assistance_request", "std_msgs/msg/String"),
}


def run_ros_command(args, timeout=5.0):
    completed = subprocess.run(
        args,
        check=False,
        timeout=timeout,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())
    return completed.stdout.strip()


def extract_ros_value(output):
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("data:"):
            return stripped.split("data:", 1)[1].strip().strip("'\"")
    return output.strip()


def topic_poll_worker():
    while True:
        for key, (topic, _msg_type) in TOPICS.items():
            try:
                output = run_ros_command(
                    ["ros2", "topic", "echo", topic, "--once", "--no-daemon"],
                    timeout=2.5,
                )
                CACHE.set(key, extract_ros_value(output))
            except Exception as exc:
                CACHE.set(key, f"unavailable: {exc}")
        time.sleep(0.3)


def publish_mission_command(command):
    payload = "{data: " + command + "}"
    run_ros_command(
        [
            "ros2",
            "topic",
            "pub",
            "/mission/command",
            "std_msgs/msg/String",
            payload,
            "--once",
        ],
        timeout=5.0,
    )


def publish_mission_decision(decision):
    payload = "{data: " + decision + "}"
    run_ros_command(
        [
            "ros2",
            "topic",
            "pub",
            "/mission/decision",
            "std_msgs/msg/String",
            payload,
            "--once",
        ],
        timeout=5.0,
    )


class ControlPanelHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/status":
            self.write_json(CACHE.snapshot())
            return
        if path == "/":
            self.serve_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
            return
        if path == "/app.css":
            self.serve_file(STATIC_DIR / "app.css", "text/css; charset=utf-8")
            return
        if path == "/app.js":
            self.serve_file(STATIC_DIR / "app.js", "application/javascript")
            return
        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/api/mission", "/api/decision"):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        try:
            data = json.loads(body or "{}")
            if path == "/api/mission":
                command = str(data.get("command", "")).strip().lower()
                if command not in {
                    "patrol",
                    "delivery",
                    "guide",
                    "evacuate",
                    "return_home",
                    "alert",
                    "stop",
                    "next",
                    "reset_patrol",
                }:
                    raise ValueError(f"unsupported command: {command}")
                publish_mission_command(command)
                self.write_json({"ok": True, "command": command})
                return

            decision = str(data.get("decision", "")).strip().lower()
            if decision not in {"wait", "retry", "next", "stop", "alert"}:
                raise ValueError(f"unsupported decision: {decision}")
            publish_mission_decision(decision)
            self.write_json({"ok": True, "decision": decision})
        except Exception as exc:
            self.write_json({"ok": False, "error": str(exc)}, status=400)

    def serve_file(self, path, content_type):
        if not path.exists():
            self.send_error(404)
            return
        content = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def write_json(self, payload, status=200):
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, fmt, *args):
        return


def main():
    threading.Thread(target=topic_poll_worker, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", DEFAULT_PORT), ControlPanelHandler)
    print(f"Camping control panel: http://localhost:{DEFAULT_PORT}")
    print(f"Camera stream: {CAMERA_STREAM_URL}")
    server.serve_forever()


if __name__ == "__main__":
    main()
