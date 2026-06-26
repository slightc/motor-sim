# -*- coding: utf-8 -*-
"""
交互式仿真界面后端（纯标准库 http.server）。

路由：
  GET  /                  -> 单文件 UI（static/index.html）
  GET  /api/channels      -> 通道与控制器/逆变器元数据
  GET  /api/status        -> 当前运行状态
  POST /api/config        -> 切换控制器/逆变器/母线电压/限流（会复位）
  POST /api/cmd           -> 启停/给定/负载/速度倍率/暂停
  POST /api/data          -> {t0,t1,width,keys,live} -> 抽稀后的窗口数据 + 状态

运行：  python3 interactive/server.py            （默认 http://127.0.0.1:8000）
        python3 interactive/server.py 8080 0.0.0.0
"""
import sys, os, json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(__file__))
from sim_engine import SimEngine, CHANNELS, CONTROLLERS, INVERTERS, CHANNEL_KEYS

HERE = os.path.dirname(os.path.abspath(__file__))
INDEX = os.path.join(HERE, "static", "index.html")

engine = SimEngine(window_s=10.0)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass  # 静音访问日志

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
        except Exception:
            return {}

    # ---------- GET ----------
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            try:
                with open(INDEX, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(500, "index.html not found", "text/plain")
            return
        if path == "/api/channels":
            self._send(200, {
                "channels": [
                    {"key": k, "label": lab, "unit": u, "group": g, "color": col}
                    for (k, lab, u, g, col) in CHANNELS
                ],
                "controllers": [{"key": k, "label": v[0]} for k, v in CONTROLLERS.items()],
                "inverters": [{"key": k, "label": v} for k, v in INVERTERS.items()],
            })
            return
        if path == "/api/status":
            self._send(200, engine.status())
            return
        self._send(404, {"error": "not found"})

    # ---------- POST ----------
    def do_POST(self):
        path = self.path.split("?", 1)[0]
        body = self._read_json()
        if path == "/api/config":
            engine.apply_config(
                controller=body.get("controller"),
                inverter=body.get("inverter"),
                v_dc=body.get("v_dc"),
                i_max=body.get("i_max"),
            )
            self._send(200, engine.status())
            return
        if path == "/api/cmd":
            action = body.get("action")
            if action == "start":
                engine.set_cmd(enabled=True)
            elif action == "stop":
                engine.set_cmd(enabled=False)
            elif action == "reset":
                engine.reset()
            elif action == "pause":
                engine.set_cmd(paused=True)
            elif action == "resume":
                engine.set_cmd(paused=False)
            engine.set_cmd(
                ref=body.get("ref"),
                load=body.get("load"),
                speed=body.get("speed"),
            )
            self._send(200, engine.status())
            return
        if path == "/api/data":
            t0 = body.get("t0"); t1 = body.get("t1")
            width = int(body.get("width", 1000))
            width = max(1, min(2000, width))
            keys = body.get("keys") or CHANNEL_KEYS
            live = bool(body.get("live", False))
            span = body.get("span", engine.window_s)
            head, count, N, oldest, latest = engine.ring.snapshot_meta()
            if live or t0 is None or t1 is None:
                t1 = latest
                t0 = max(oldest, latest - float(span))
            series, oldest, latest = engine.query(float(t0), float(t1), keys, width)
            self._send(200, {
                "t0": t0, "t1": t1, "width": width,
                "series": series,
                "status": engine.status(),
            })
            return
        self._send(404, {"error": "not found"})


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
    host = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
    engine.start()
    srv = ThreadingHTTPServer((host, port), Handler)
    print(f"电机交互仿真界面: http://{host}:{port}  (Ctrl-C 退出)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n已退出")


if __name__ == "__main__":
    main()
