"""Zero-dependency Model Context Protocol (MCP) server for LinuxCNC PID & Feedforward Auto-Tuner.

Supports:
  1. Standard MCP stdio JSON-RPC 2.0 transport (`python3 main.py --mcp`).
  2. Optional HTTP JSON-RPC transport (`python3 main.py --mcp-http --port 8765`).

Allows an AI assistant to inspect axis parameters, execute safe G-code tuning runs,
analyze oscilloscope KPIs, and send modified PID/FF parameters back with strict
safety validation before applying them to LinuxCNC HAL or `.ini` files.
"""

from __future__ import annotations

import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, TextIO

from .gcode_gen import generate_tuning_gcode
from .hal_backend import AXIS_TO_JOINT, LinuxCNCBackend
from .kpi_analyzer import analyze_trace
from .models import MotionConfig, PIDParams, TuningAttempt
from .optimizer import PIDAutoOptimizer
from .validator import validate_pid_params


MCP_PROTOCOL_VERSION = "2024-11-05"
SERVER_NAME = "linuxcnc-pid-autotuner"
SERVER_VERSION = "1.0.0"


class PIDTunerMCPServer:
    """Handles MCP JSON-RPC 2.0 requests for LinuxCNC PID tuning."""

    def __init__(self, force_sim: bool = False) -> None:
        self.backend = LinuxCNCBackend(force_sim=force_sim)
        self.optimizer = PIDAutoOptimizer()
        self.attempts: List[TuningAttempt] = []
        self._tool_handlers: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
            "get_axis_params": self._tool_get_axis_params,
            "run_tuning_test": self._tool_run_tuning_test,
            "validate_and_apply_params": self._tool_validate_and_apply_params,
            "get_tuning_history": self._tool_get_tuning_history,
            "save_params_to_ini": self._tool_save_params_to_ini,
        }

    # ------------------------------------------------------------------
    # Tool Definitions (JSON Schema)
    # ------------------------------------------------------------------
    def list_tools(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": "get_axis_params",
                "description": (
                    "Read active PID & Feedforward parameters (P, I, D, FF0, FF1, FF2, "
                    "deadband, bias, maxerrorI, maxoutput), soft limits, and live/sim status "
                    "for a LinuxCNC axis."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "axis": {
                            "type": "string",
                            "enum": ["X", "Y", "Z", "A"],
                            "default": "X",
                            "description": "Target LinuxCNC axis letter.",
                        }
                    },
                },
            },
            {
                "name": "run_tuning_test",
                "description": (
                    "Execute a safe back-and-forth tuning G-code move on the specified axis, "
                    "record HAL oscilloscope signals, compute all motion-phase KPIs (RMS/Peak "
                    "ferror, cruise lag, accel/decel spikes, overshoot, settling time, "
                    "standstill hunting, drive chatter), and return results + baseline suggestion."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "axis": {
                            "type": "string",
                            "enum": ["X", "Y", "Z", "A"],
                            "default": "X",
                        },
                        "start_pos": {
                            "type": "number",
                            "description": "Start position in mm (defaults to current live position or 0.0).",
                        },
                        "travel_distance": {
                            "type": "number",
                            "default": 30.0,
                            "description": "Signed travel distance in mm (negative flips direction).",
                        },
                        "cruise_feedrate": {
                            "type": "number",
                            "default": 1500.0,
                            "description": "Feedrate in mm/min.",
                        },
                        "max_acceleration": {
                            "type": "number",
                            "default": 250.0,
                            "description": "Expected axis acceleration in mm/s^2.",
                        },
                        "dwell_time": {
                            "type": "number",
                            "default": 0.35,
                            "description": "Dwell duration in seconds at each end.",
                        },
                    },
                },
            },
            {
                "name": "validate_and_apply_params",
                "description": (
                    "Validate AI-modified PID & Feedforward parameters against hard safety "
                    "bounds, velocity-servo/linear-scale checks, and single-step jump limits. "
                    "If valid and apply=true, writes parameters to LinuxCNC HAL and optionally "
                    "runs a verification G-code test immediately."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "axis": {
                            "type": "string",
                            "enum": ["X", "Y", "Z", "A"],
                            "default": "X",
                        },
                        "params": {
                            "type": "object",
                            "description": "Modified parameters dict (any subset of P, I, D, FF0, FF1, FF2, deadband, bias, maxerrorI, maxoutput).",
                            "additionalProperties": {"type": "number"},
                        },
                        "apply": {
                            "type": "boolean",
                            "default": True,
                            "description": "Write validated parameters to HAL if valid.",
                        },
                        "strict_step_limit": {
                            "type": "boolean",
                            "default": True,
                            "description": "Enforce single-step gain jump limits (max 3x P/I, 5x D, 2.5x FF1).",
                        },
                        "run_test_after": {
                            "type": "boolean",
                            "default": False,
                            "description": "If true and apply=true, run a verification G-code test immediately.",
                        },
                        "travel_distance": {
                            "type": "number",
                            "default": 30.0,
                        },
                        "cruise_feedrate": {
                            "type": "number",
                            "default": 1500.0,
                        },
                        "max_acceleration": {
                            "type": "number",
                            "default": 250.0,
                        },
                    },
                    "required": ["params"],
                },
            },
            {
                "name": "get_tuning_history",
                "description": (
                    "Return the recorded succession of tuning attempts (parameters, KPIs, "
                    "and scores) so the AI can track improvement over iterations."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "axis": {
                            "type": "string",
                            "enum": ["X", "Y", "Z", "A"],
                            "description": "Optional axis filter.",
                        }
                    },
                },
            },
            {
                "name": "save_params_to_ini",
                "description": (
                    "Save the active validated PID & Feedforward parameters to a LinuxCNC .ini "
                    "file after creating a timestamped .bak backup."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "axis": {
                            "type": "string",
                            "enum": ["X", "Y", "Z", "A"],
                            "default": "X",
                        },
                        "ini_path": {
                            "type": "string",
                            "description": "Path to the LinuxCNC .ini file (optional if INI_FILE_NAME is set).",
                        },
                    },
                },
            },
        ]

    # ------------------------------------------------------------------
    # Tool Implementations
    # ------------------------------------------------------------------
    def _tool_get_axis_params(self, args: Dict[str, Any]) -> Dict[str, Any]:
        axis = str(args.get("axis", "X")).upper()
        params = self.backend.read_pid_params(axis)
        min_lim, max_lim = self.backend.read_axis_limits(axis)
        cur_pos = self.backend.read_current_position(axis)
        return {
            "axis": axis,
            "joint_index": AXIS_TO_JOINT.get(axis, 0),
            "mode": "live_linuxcnc" if self.backend.is_live_linuxcnc else "simulation",
            "current_position_mm": cur_pos,
            "soft_limits_mm": {"min": min_lim, "max": max_lim},
            "params": params.to_dict(),
        }

    def _tool_run_tuning_test(self, args: Dict[str, Any]) -> Dict[str, Any]:
        axis = str(args.get("axis", "X")).upper()
        start_pos = args.get("start_pos")
        if start_pos is None:
            start_pos = self.backend.read_current_position(axis)
        else:
            start_pos = float(start_pos)

        travel = float(args.get("travel_distance", 30.0))
        feed = float(args.get("cruise_feedrate", 1500.0))
        accel = float(args.get("max_acceleration", 250.0))
        dwell = float(args.get("dwell_time", 0.35))

        is_safe, safe_travel, limit_msg = self.backend.validate_travel_within_limits(
            axis, start_pos, travel
        )
        if not is_safe:
            return {
                "error": "soft_limit_violation",
                "message": limit_msg,
                "suggested_safe_travel_mm": safe_travel,
            }

        cfg = MotionConfig(
            axis_letter=axis,
            joint_index=AXIS_TO_JOINT.get(axis, 0),
            start_pos=start_pos,
            travel_distance=travel,
            cruise_feedrate=feed,
            max_acceleration=accel,
            dwell_time=dwell,
        )
        params = self.backend.read_pid_params(axis)
        trace = self.backend.run_test_and_record(cfg, params)
        kpis = analyze_trace(trace, params)

        axis_history = [a for a in self.attempts if a.axis == axis]
        suggestion = self.optimizer.suggest_next(
            params, kpis, cfg, history=axis_history
        )
        attempt_id = len(self.attempts) + 1
        attempt = TuningAttempt.create(
            attempt_id=attempt_id,
            axis=axis,
            stage_name=suggestion.stage_name,
            params=params,
            kpis=kpis,
            suggestion=suggestion,
            trace=trace,
            gcode_used=generate_tuning_gcode(cfg),
        )
        self.attempts.append(attempt)

        return {
            "attempt_id": attempt.attempt_id,
            "axis": axis,
            "mode": "live_linuxcnc" if self.backend.is_live_linuxcnc else "simulation",
            "params_tested": params.to_dict(),
            "kpis": kpis.to_dict(),
            "kpis_um": {
                "ferror_rms_um": round(kpis.ferror_rms * 1000.0, 2),
                "ferror_max_um": round(kpis.ferror_max * 1000.0, 2),
                "cruise_error_mean_um": round(kpis.cruise_error_mean * 1000.0, 2),
                "accel_peak_error_um": round(kpis.accel_peak_error * 1000.0, 2),
                "overshoot_um": round(kpis.overshoot_mm * 1000.0, 2),
                "standstill_hunting_um": round(kpis.standstill_hunting_p2p * 1000.0, 2),
            },
            "baseline_optimizer_suggestion": {
                "stage_name": suggestion.stage_name,
                "suggested_params": suggestion.suggested_params.to_dict(),
                "rationale": suggestion.rationale,
                "warnings": suggestion.warnings,
            },
        }

    def _tool_validate_and_apply_params(self, args: Dict[str, Any]) -> Dict[str, Any]:
        axis = str(args.get("axis", "X")).upper()
        proposed = args.get("params", {})
        apply_flag = bool(args.get("apply", True))
        strict_step = bool(args.get("strict_step_limit", True))
        run_test_after = bool(args.get("run_test_after", False))

        current = self.backend.read_pid_params(axis)
        val_res = validate_pid_params(
            proposed=proposed,
            current=current,
            strict_step_limit=strict_step,
        )

        response: Dict[str, Any] = {
            "axis": axis,
            "validation": val_res.to_dict(),
            "applied": False,
        }

        if not val_res.valid:
            response["message"] = (
                "Validation failed; parameters were NOT applied to HAL."
            )
            return response

        if apply_flag:
            ok, msg = self.backend.write_pid_params(axis, val_res.validated_params)
            response["applied"] = ok
            response["apply_message"] = msg

            if ok and run_test_after:
                test_res = self._tool_run_tuning_test(
                    {
                        "axis": axis,
                        "travel_distance": args.get("travel_distance", 30.0),
                        "cruise_feedrate": args.get("cruise_feedrate", 1500.0),
                        "max_acceleration": args.get("max_acceleration", 250.0),
                    }
                )
                response["verification_test"] = test_res

        return response

    def _tool_get_tuning_history(self, args: Dict[str, Any]) -> Dict[str, Any]:
        axis_filter = args.get("axis")
        rows = []
        for a in self.attempts:
            if axis_filter and a.axis != str(axis_filter).upper():
                continue
            rows.append(
                {
                    "attempt_id": a.attempt_id,
                    "timestamp": a.timestamp,
                    "axis": a.axis,
                    "stage_name": a.stage_name,
                    "params": a.params.to_dict(),
                    "kpis": a.kpis.to_dict(),
                }
            )
        return {"count": len(rows), "attempts": rows}

    def _tool_save_params_to_ini(self, args: Dict[str, Any]) -> Dict[str, Any]:
        axis = str(args.get("axis", "X")).upper()
        ini_str = args.get("ini_path")
        ini_path = Path(ini_str) if ini_str else self.backend.ini_path
        if ini_path is None:
            return {
                "saved": False,
                "error": "No ini_path provided and INI_FILE_NAME environment variable is not set.",
            }
        params = self.backend.read_pid_params(axis)
        val_res = validate_pid_params(params.to_dict(), current=None, strict_step_limit=False)
        if not val_res.valid:
            return {
                "saved": False,
                "error": "Current parameters failed safety validation.",
                "validation": val_res.to_dict(),
            }
        ok, msg = self.backend.save_params_to_ini(ini_path, axis, params)
        return {"saved": ok, "message": msg, "ini_path": str(ini_path)}

    # ------------------------------------------------------------------
    # Resources
    # ------------------------------------------------------------------
    def list_resources(self) -> List[Dict[str, Any]]:
        resources = [
            {
                "uri": "lcnc://session/history",
                "name": "Tuning Session Attempt History",
                "mimeType": "application/json",
            }
        ]
        for ax in ("X", "Y", "Z", "A"):
            resources.append(
                {
                    "uri": f"lcnc://axis/{ax}/params",
                    "name": f"Axis {ax} Active PID & Feedforward Parameters",
                    "mimeType": "application/json",
                }
            )
        return resources

    def read_resource(self, uri: str) -> str:
        if uri == "lcnc://session/history":
            return json.dumps(self._tool_get_tuning_history({}), indent=2)
        if uri.startswith("lcnc://axis/") and uri.endswith("/params"):
            ax = uri.split("/")[3].upper()
            return json.dumps(self._tool_get_axis_params({"axis": ax}), indent=2)
        raise ValueError(f"Unknown resource URI: {uri}")

    # ------------------------------------------------------------------
    # JSON-RPC 2.0 Dispatcher
    # ------------------------------------------------------------------
    def handle_request(self, req: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Process a single JSON-RPC 2.0 message and return response dict (or None for notifications)."""
        req_id = req.get("id")
        method = req.get("method", "")
        params = req.get("params") or {}

        # Notifications have no id and expect no response
        if method.startswith("notifications/") and req_id is None:
            return None

        try:
            if method == "initialize":
                result: Any = {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities": {
                        "tools": {"listChanged": False},
                        "resources": {"subscribe": False, "listChanged": False},
                    },
                    "serverInfo": {
                        "name": SERVER_NAME,
                        "version": SERVER_VERSION,
                    },
                }
            elif method == "ping":
                result = {}
            elif method == "tools/list":
                result = {"tools": self.list_tools()}
            elif method == "tools/call":
                tool_name = params.get("name", "")
                tool_args = params.get("arguments") or {}
                handler = self._tool_handlers.get(tool_name)
                if handler is None:
                    return self._error_response(
                        req_id, -32601, f"Unknown tool: {tool_name}"
                    )
                tool_output = handler(tool_args)
                result = {
                    "content": [
                        {
                            "type": "text",
                            "text": json.dumps(tool_output, indent=2),
                        }
                    ],
                    "isError": bool("error" in tool_output and tool_output.get("applied") is not True),
                }
            elif method == "resources/list":
                result = {"resources": self.list_resources()}
            elif method == "resources/read":
                uri = params.get("uri", "")
                text = self.read_resource(uri)
                result = {
                    "contents": [
                        {
                            "uri": uri,
                            "mimeType": "application/json",
                            "text": text,
                        }
                    ]
                }
            else:
                if req_id is None:
                    return None
                return self._error_response(
                    req_id, -32601, f"Method not found: {method}"
                )

            if req_id is None:
                return None
            return {"jsonrpc": "2.0", "id": req_id, "result": result}
        except Exception as exc:
            if req_id is None:
                return None
            return self._error_response(req_id, -32603, str(exc))

    @staticmethod
    def _error_response(req_id: Any, code: int, message: str) -> Dict[str, Any]:
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": code, "message": message},
        }

    # ------------------------------------------------------------------
    # Transports: stdio & HTTP
    # ------------------------------------------------------------------
    def run_stdio(
        self,
        in_stream: Optional[TextIO] = None,
        out_stream: Optional[TextIO] = None,
    ) -> None:
        """Run blocking MCP JSON-RPC 2.0 loop over stdin/stdout."""
        fin = in_stream or sys.stdin
        fout = out_stream or sys.stdout

        for raw_line in fin:
            line = raw_line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except json.JSONDecodeError as exc:
                err = self._error_response(None, -32700, f"Parse error: {exc}")
                fout.write(json.dumps(err) + "\n")
                fout.flush()
                continue

            resp = self.handle_request(req)
            if resp is not None:
                fout.write(json.dumps(resp) + "\n")
                fout.flush()

    def run_http(self, host: str = "127.0.0.1", port: int = 8765) -> None:
        """Run a lightweight HTTP JSON-RPC server (`POST /mcp`) for LAN/local AI integration."""
        mcp_instance = self

        class MCPHttpHandler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length).decode("utf-8", errors="replace")
                try:
                    req = json.loads(body)
                    resp = mcp_instance.handle_request(req)
                except Exception as exc:
                    resp = mcp_instance._error_response(None, -32700, str(exc))

                payload = json.dumps(resp or {}).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, format: str, *args: object) -> None:
                # Keep stdout/stderr clean unless needed
                pass

        server = ThreadingHTTPServer((host, port), MCPHttpHandler)
        print(
            f"LinuxCNC PID Auto-Tuner MCP HTTP server listening on http://{host}:{port}/mcp",
            file=sys.stderr,
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.shutdown()


def run_mcp_stdio(force_sim: bool = False) -> None:
    server = PIDTunerMCPServer(force_sim=force_sim)
    server.run_stdio()


def run_mcp_http(
    host: str = "127.0.0.1", port: int = 8765, force_sim: bool = False
) -> None:
    server = PIDTunerMCPServer(force_sim=force_sim)
    server.run_http(host=host, port=port)
