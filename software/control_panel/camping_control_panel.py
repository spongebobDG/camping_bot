#!/usr/bin/env python3
import ast
import base64
import json
import math
import os
import re
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
MAP_YAML_PATH = os.environ.get(
    "CAMPING_MAP_YAML", "/home/spbdg/maps/camping_test_map.yaml"
)


POSITION_RE = re.compile(
    r"position:\s*\n\s*x:\s*(?P<x>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\n"
    r"\s*y:\s*(?P<y>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\n"
    r"\s*z:\s*(?P<z>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
)
ORIENTATION_RE = re.compile(
    r"orientation:\s*\n\s*x:\s*(?P<x>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\n"
    r"\s*y:\s*(?P<y>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\n"
    r"\s*z:\s*(?P<z>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\n"
    r"\s*w:\s*(?P<w>-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
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
            "battery_status": "unknown",
            "assistance_request": "none",
            "elevator_status": "phase=idle",
            "robot_pose": None,
            "goal_pose": None,
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
MAP_CACHE = {"path": None, "mtime": None, "data": None}


TOPICS = {
    "mission_status": ("/mission/status", "std_msgs/msg/String"),
    "mission_level": ("/mission/level", "std_msgs/msg/String"),
    "mission_mode_status": ("/mission/mode_status", "std_msgs/msg/String"),
    "mission_task_status": ("/mission/task_status", "std_msgs/msg/String"),
    "camera_status": ("/camera/status", "std_msgs/msg/String"),
    "camera_online": ("/camera/online", "std_msgs/msg/Bool"),
    "hazard": ("/camping_robot/hazard", "std_msgs/msg/String"),
    "battery_status": ("/battery/status", "std_msgs/msg/String"),
    "assistance_request": ("/mission/assistance_request", "std_msgs/msg/String"),
    "elevator_status": ("/mission/elevator_status", "std_msgs/msg/String"),
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


def parse_pose_text(output):
    position = POSITION_RE.search(output)
    orientation = ORIENTATION_RE.search(output)
    if not position or not orientation:
        return None

    px = float(position.group("x"))
    py = float(position.group("y"))
    qx = float(orientation.group("x"))
    qy = float(orientation.group("y"))
    qz = float(orientation.group("z"))
    qw = float(orientation.group("w"))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {"x": px, "y": py, "yaw": yaw}


def yaw_to_quaternion(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def publish_goal_pose(x, y, yaw):
    qz, qw = yaw_to_quaternion(yaw)
    payload = (
        "{header: {frame_id: map}, "
        "pose: {"
        f"position: {{x: {x:.4f}, y: {y:.4f}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}"
        "}}"
    )
    run_ros_command(
        [
            "ros2",
            "topic",
            "pub",
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            payload,
            "--once",
        ],
        timeout=5.0,
    )
    CACHE.set("goal_pose", {"x": x, "y": y, "yaw": yaw})


def publish_initial_pose(x, y, yaw):
    qz, qw = yaw_to_quaternion(yaw)
    covariance = [0.0] * 36
    covariance[0] = 0.25
    covariance[7] = 0.25
    covariance[35] = 0.06853891909122467
    payload = (
        "{header: {frame_id: map}, "
        "pose: {pose: {"
        f"position: {{x: {x:.4f}, y: {y:.4f}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}"
        f"}}, covariance: {covariance}}}"
    )
    run_ros_command(
        [
            "ros2",
            "topic",
            "pub",
            "/initialpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            payload,
            "--once",
        ],
        timeout=5.0,
    )
    CACHE.set("robot_pose", {"x": x, "y": y, "yaw": yaw})


def nav_poll_worker():
    while True:
        try:
            output = run_ros_command(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "/amcl_pose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    "--once",
                    "--no-daemon",
                ],
                timeout=1.8,
            )
            pose = parse_pose_text(output)
            if pose:
                CACHE.set("robot_pose", pose)
        except Exception:
            pass

        try:
            output = run_ros_command(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "/goal_pose",
                    "geometry_msgs/msg/PoseStamped",
                    "--once",
                    "--no-daemon",
                ],
                timeout=0.8,
            )
            pose = parse_pose_text(output)
            if pose:
                CACHE.set("goal_pose", pose)
        except Exception:
            pass

        time.sleep(0.4)


def parse_map_yaml(path):
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip().lstrip("\ufeff")] = value.strip().strip("'\"")

    image = values.get("image")
    if not image:
        raise ValueError(f"map yaml has no image: {path}")

    image_path = Path(image).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path

    origin = ast.literal_eval(values.get("origin", "[0, 0, 0]"))
    return {
        "image_path": image_path,
        "resolution": float(values.get("resolution", "0.05")),
        "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
        "occupied_thresh": float(values.get("occupied_thresh", "0.65")),
        "free_thresh": float(values.get("free_thresh", "0.25")),
    }


def read_pgm(path):
    with path.open("rb") as f:
        def token():
            data = bytearray()
            while True:
                ch = f.read(1)
                if not ch:
                    raise ValueError("unexpected end of PGM header")
                if ch == b"#":
                    f.readline()
                    continue
                if ch.isspace():
                    continue
                data.extend(ch)
                break
            while True:
                ch = f.read(1)
                if not ch or ch.isspace():
                    break
                data.extend(ch)
            return bytes(data).decode("ascii")

        magic = token()
        if magic != "P5":
            raise ValueError(f"unsupported map image format {magic}; expected P5 PGM")
        width = int(token())
        height = int(token())
        max_value = int(token())
        if max_value > 255:
            raise ValueError("16-bit PGM maps are not supported")
        pixels = f.read(width * height)
        if len(pixels) != width * height:
            raise ValueError("PGM pixel data is shorter than expected")
        return width, height, pixels


def load_map_payload():
    yaml_path = Path(MAP_YAML_PATH).expanduser()
    if not yaml_path.exists():
        raise FileNotFoundError(f"map yaml not found: {yaml_path}")

    mtime = yaml_path.stat().st_mtime
    cached = MAP_CACHE["data"]
    if MAP_CACHE["path"] == str(yaml_path) and MAP_CACHE["mtime"] == mtime and cached:
        return cached

    metadata = parse_map_yaml(yaml_path)
    width, height, pixels = read_pgm(metadata["image_path"])
    payload = {
        "ok": True,
        "yaml_path": str(yaml_path),
        "image_path": str(metadata["image_path"]),
        "width": width,
        "height": height,
        "resolution": metadata["resolution"],
        "origin": metadata["origin"],
        "pixels_b64": base64.b64encode(pixels).decode("ascii"),
    }
    MAP_CACHE.update({"path": str(yaml_path), "mtime": mtime, "data": payload})
    return payload


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


def publish_elevator_decision(decision):
    payload = "{data: " + decision + "}"
    run_ros_command(
        [
            "ros2",
            "topic",
            "pub",
            "/mission/elevator_decision",
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
        if path == "/api/map":
            try:
                self.write_json(load_map_payload())
            except Exception as exc:
                self.write_json({"ok": False, "error": str(exc)}, status=404)
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
        if path not in (
            "/api/mission",
            "/api/decision",
            "/api/elevator",
            "/api/goal",
            "/api/initialpose",
        ):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        try:
            data = json.loads(body or "{}")
            if path in ("/api/goal", "/api/initialpose"):
                x = float(data["x"])
                y = float(data["y"])
                yaw = float(data.get("yaw", 0.0))
                if path == "/api/goal":
                    publish_goal_pose(x, y, yaw)
                    self.write_json({"ok": True, "target": "goal", "x": x, "y": y, "yaw": yaw})
                    return
                publish_initial_pose(x, y, yaw)
                self.write_json({"ok": True, "target": "initialpose", "x": x, "y": y, "yaw": yaw})
                return

            if path == "/api/mission":
                command = str(data.get("command", "")).strip().lower()
                if command not in {
                    "patrol",
                    "delivery",
                    "guide",
                    "evacuate",
                    "elevator",
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
            if path == "/api/elevator":
                if decision not in {
                    "call",
                    "entered",
                    "floor_selected",
                    "exited",
                    "complete",
                    "cancel",
                }:
                    raise ValueError(f"unsupported elevator decision: {decision}")
                publish_elevator_decision(decision)
                self.write_json({"ok": True, "decision": decision})
                return

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
    threading.Thread(target=nav_poll_worker, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", DEFAULT_PORT), ControlPanelHandler)
    print(f"Camping control panel: http://localhost:{DEFAULT_PORT}")
    print(f"Camera stream: {CAMERA_STREAM_URL}")
    print(f"Map YAML: {MAP_YAML_PATH}")
    server.serve_forever()


if __name__ == "__main__":
    main()
