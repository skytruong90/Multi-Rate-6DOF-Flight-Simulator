from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class State:
    t: float = 0.0
    altitude_m: float = 1000.0
    speed_mps: float = 120.0
    roll_deg: float = 0.0
    pitch_deg: float = 2.0
    yaw_deg: float = 0.0
    roll_rate_dps: float = 0.0
    pitch_rate_dps: float = 0.0
    yaw_rate_dps: float = 0.0


@dataclass
class Command:
    altitude_m: float = 1500.0
    speed_mps: float = 150.0
    heading_deg: float = 20.0


class MultiRateSimulator:
    def __init__(self, dynamics_hz: int = 100, control_hz: int = 20, sensor_hz: int = 10):
        if not (dynamics_hz >= control_hz >= sensor_hz > 0):
            raise ValueError("rates must satisfy dynamics >= control >= sensor > 0")
        self.dynamics_hz = dynamics_hz
        self.control_hz = control_hz
        self.sensor_hz = sensor_hz
        self.dt = 1.0 / dynamics_hz
        self.control_stride = dynamics_hz // control_hz
        self.sensor_stride = dynamics_hz // sensor_hz
        self.state = State()
        self.command = Command()
        self.control = {"throttle": 0.5, "roll_cmd": 0.0, "pitch_cmd": 0.0, "yaw_cmd": 0.0}
        self.telemetry: list[dict[str, float]] = []

    @staticmethod
    def _wrap_angle(deg: float) -> float:
        return (deg + 180.0) % 360.0 - 180.0

    def _controller_step(self) -> None:
        s, c = self.state, self.command
        alt_error = c.altitude_m - s.altitude_m
        speed_error = c.speed_mps - s.speed_mps
        heading_error = self._wrap_angle(c.heading_deg - s.yaw_deg)
        self.control["throttle"] = max(0.0, min(1.0, 0.5 + 0.01 * speed_error))
        self.control["pitch_cmd"] = max(-12.0, min(12.0, 0.015 * alt_error))
        self.control["roll_cmd"] = max(-25.0, min(25.0, 0.6 * heading_error))
        self.control["yaw_cmd"] = max(-8.0, min(8.0, 0.2 * heading_error))

    def _dynamics_step(self) -> None:
        s = self.state
        gust = 2.5 * math.sin(0.7 * s.t) if 6.0 <= s.t <= 10.0 else 0.0
        throttle = self.control["throttle"]
        drag = 0.0026 * s.speed_mps * s.speed_mps
        thrust_accel = 58.0 * throttle
        s.speed_mps = max(30.0, s.speed_mps + (thrust_accel - drag + 0.08 * gust) * self.dt)

        for angle, rate, command, damping in (
            ("roll_deg", "roll_rate_dps", "roll_cmd", 1.8),
            ("pitch_deg", "pitch_rate_dps", "pitch_cmd", 2.2),
            ("yaw_deg", "yaw_rate_dps", "yaw_cmd", 1.5),
        ):
            error = self.control[command] - getattr(s, angle)
            rate_dot = 3.0 * error - damping * getattr(s, rate)
            setattr(s, rate, getattr(s, rate) + rate_dot * self.dt)
            setattr(s, angle, getattr(s, angle) + getattr(s, rate) * self.dt)

        climb_rate = s.speed_mps * math.sin(math.radians(s.pitch_deg)) + 0.2 * gust
        s.altitude_m = max(0.0, s.altitude_m + climb_rate * self.dt)
        s.yaw_deg = self._wrap_angle(s.yaw_deg)
        s.t += self.dt

    def _sensor_step(self) -> None:
        s = self.state
        self.telemetry.append({
            "t": round(s.t, 6),
            "altitude_m": s.altitude_m,
            "speed_mps": s.speed_mps,
            "roll_deg": s.roll_deg,
            "pitch_deg": s.pitch_deg,
            "yaw_deg": s.yaw_deg,
        })

    def run(self, duration_s: float) -> list[dict[str, float]]:
        steps = int(round(duration_s * self.dynamics_hz))
        for step in range(steps):
            if step % self.control_stride == 0:
                self._controller_step()
            self._dynamics_step()
            if step % self.sensor_stride == 0:
                self._sensor_step()
        return self.telemetry

    def summary(self) -> dict[str, float | int]:
        s = self.state
        return {
            "duration_s": round(s.t, 3),
            "samples": len(self.telemetry),
            "final_altitude_m": round(s.altitude_m, 2),
            "final_speed_mps": round(s.speed_mps, 2),
            "final_heading_deg": round(s.yaw_deg, 2),
            "dynamics_hz": self.dynamics_hz,
            "control_hz": self.control_hz,
            "sensor_hz": self.sensor_hz,
        }


def write_outputs(sim: MultiRateSimulator, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    rows = sim.telemetry
    if rows:
        with (output / "telemetry.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(sim.summary(), indent=2), encoding="utf-8")


def main() -> None:
    p = argparse.ArgumentParser(description="Synthetic multi-rate flight simulator")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--output", type=Path, default=Path("artifacts"))
    args = p.parse_args()
    sim = MultiRateSimulator()
    sim.run(args.duration)
    write_outputs(sim, args.output)
    print(json.dumps(sim.summary(), indent=2))


if __name__ == "__main__":
    main()
