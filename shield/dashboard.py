"""Local dashboard: FastAPI + uvicorn on a background thread. Open http://127.0.0.1:8000.

    GET  /             the page (static/index.html, app.js, style.css; no CDN, no build)
    WS   /ws           state pushed at ~10 Hz
    GET  /stream       MJPEG camera preview with detection overlays
    POST /api/coverage {"value": 0..1}      request a coverage
    POST /api/open, /api/close              request 0 / 1
    POST /api/emergency                     emergency close (a hazard: it latches)
    POST /hazard       {level, kind, confidence, ttc_s, source}   only if external_hazards

Authority stays with the control loop. Everything here is a *request* that goes through
the same Arbiter: the physical override always wins, a hazard close beats a dashboard OPEN,
and nothing here can touch the watchdog, stall detection or camera fail-safe. If this
thread dies, control carries on.
"""
import asyncio
import collections
import json
import logging
import math
import threading
import time
from pathlib import Path

from .state import Hazard

log = logging.getLogger(__name__)
STATIC = Path(__file__).with_name("static")


def _clean(x):
    """JSON-safe: inf/nan -> None, recursively."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    return x


class Hub:
    """Thread-safe bridge between the control loop and the dashboard."""

    def __init__(self):
        self._lock = threading.Lock()
        self._status = "{}"
        self.events = collections.deque(maxlen=60)
        self.request = None          # requested coverage, or None = rest open
        self._emergency = False
        self._external = None

    # control loop side
    def publish(self, status: dict) -> None:
        with self._lock:
            status["events"] = list(self.events)
            self._status = json.dumps(_clean(status))

    def take_hazard(self) -> Hazard | None:
        """A dashboard EMERGENCY CLOSE or an external hazard, consumed once."""
        with self._lock:
            if self._emergency:
                self._emergency = False
                return Hazard(1.0, True, "dashboard EMERGENCY CLOSE")
            h, self._external = self._external, None
            return h

    # dashboard side
    def snapshot(self) -> str:
        with self._lock:
            return self._status

    def set_request(self, value: float) -> None:
        self.request = min(1.0, max(0.0, float(value)))

    def emergency(self) -> None:
        with self._lock:
            self._emergency = True

    def external(self, hazard: Hazard) -> None:
        with self._lock:
            self._external = hazard

    def event(self, level: str, text: str) -> None:
        with self._lock:
            self.events.appendleft({"t": time.strftime("%H:%M:%S"), "level": level, "text": text})


class EventLogHandler(logging.Handler):
    """Mirrors shield.* log lines (INFO and up) into the dashboard's event log."""

    def __init__(self, hub: Hub):
        super().__init__(logging.INFO)
        self.hub = hub

    def emit(self, record):
        if record.name.startswith("shield") and record.name != __name__:
            try:
                self.hub.event(record.levelname, record.getMessage())
            except Exception:
                pass


def _overlay(gray, vision):
    """Grey frame -> BGR JPEG with the impact box/TTC and dust severity drawn on."""
    import cv2
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    h, w = img.shape[:2]
    d, dust = vision.detection, vision.dust
    if not vision.armed:
        cv2.putText(img, "DISARMED", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
    elif d.box is not None:
        x, y, bw, bh = d.box
        color = {"critical": (60, 60, 255), "warning": (0, 190, 255)}.get(d.level, (120, 220, 120))
        cv2.rectangle(img, (x, y), (x + bw, y + bh), color, 2)
        label = f"TTC {d.ttc:.2f}s" if math.isfinite(d.ttc) else "tracking"
        cv2.putText(img, label, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    if vision.armed:
        bar = int((w - 12) * dust.severity)
        color = {"severe": (60, 60, 255), "warning": (0, 190, 255)}.get(dust.level, (120, 220, 120))
        cv2.rectangle(img, (6, h - 10), (6 + bar, h - 5), color, -1)
        cv2.putText(img, f"dust {dust.severity:.2f}", (6, h - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, color, 1)
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpg.tobytes() if ok else None


def create_app(cfg, hub: Hub, vision):
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.responses import Response, StreamingResponse

    app = FastAPI(title="Armor-DILLO", docs_url=None, redoc_url=None)
    types = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}

    def static(name):
        path = STATIC / name
        return Response(path.read_bytes(), media_type=types[path.suffix],
                        headers={"Cache-Control": "no-cache"})

    @app.get("/")
    def index():
        return static("index.html")

    @app.get("/app.js")
    def app_js():
        return static("app.js")

    @app.get("/style.css")
    def style():
        return static("style.css")

    @app.websocket("/ws")
    async def ws(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_text(hub.snapshot())
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError, OSError):
            pass

    @app.get("/stream")
    def stream():
        period = 1.0 / cfg.dashboard.preview_fps

        def frames():  # sync generator: Starlette runs it in a worker thread
            last = None
            while True:
                gray = vision.frame
                if gray is not None and gray is not last:
                    last = gray
                    jpg = _overlay(gray, vision)
                    if jpg:
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                               + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(period)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/api/coverage")
    def coverage(body: dict):
        try:
            hub.set_request(body["value"])
        except (KeyError, TypeError, ValueError):
            raise HTTPException(400, 'expected {"value": 0..1}')
        hub.event("INFO", f"dashboard: request coverage {hub.request:.0%}")
        return {"request": hub.request}

    @app.post("/api/open")
    def open_():
        hub.set_request(0.0)
        hub.event("INFO", "dashboard: OPEN")
        return {"request": 0.0}

    @app.post("/api/close")
    def close():
        hub.set_request(1.0)
        hub.event("INFO", "dashboard: CLOSE")
        return {"request": 1.0}

    @app.post("/api/emergency")
    def emergency():
        hub.emergency()
        return {"ok": True}

    @app.post("/hazard")
    def hazard(body: dict):
        if not cfg.dashboard.external_hazards:
            raise HTTPException(403, "external hazards are disabled (dashboard.external_hazards)")
        level = str(body.get("level", "")).lower()
        if level not in ("warning", "critical"):
            raise HTTPException(400, "level must be warning or critical")
        text = (f"external {body.get('kind', 'hazard')} from {body.get('source', '?')} "
                f"(confidence {body.get('confidence', '?')}, TTC {body.get('ttc_s', '?')} s)")
        if level == "critical":
            ttc = body.get("ttc_s")
            hub.external(Hazard(1.0, True, text, float(ttc) if ttc is not None else None))
        hub.event("WARNING", f"{level.upper()}: {text}")
        return {"ok": True}

    return app


def start(cfg, hub: Hub, vision) -> None:
    """Serve on a daemon thread. Any failure is logged; control is unaffected."""
    def serve():
        try:
            import uvicorn
            app = create_app(cfg, hub, vision)
            server = uvicorn.Server(uvicorn.Config(app, host=cfg.dashboard.host,
                                                   port=cfg.dashboard.port, log_level="warning"))
            server.install_signal_handlers = lambda: None  # signals belong to the control loop
            log.info("dashboard on http://%s:%d", cfg.dashboard.host, cfg.dashboard.port)
            server.run()
        except Exception:
            log.exception("dashboard failed (control continues)")

    logging.getLogger().addHandler(EventLogHandler(hub))
    threading.Thread(target=serve, name="dashboard", daemon=True).start()
