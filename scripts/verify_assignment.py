"""
Offline verification harness for Track 1 (Critical Energy Evacuation & Handoff).

Solves the E-CARGO Group Role Assignment ILP for both demo phases using the real
`compute_q_and_bounds()` and the real PuLP/CBC solver, WITHOUT touching the Kubernetes
API -- so the expected role transitions can be rehearsed and regression-checked with no
Docker daemon or Kind cluster running.

Asserts:
  1. The Phase 1 optimum equals the declarative labels in manifests/swarm_fleet.yaml
     (proving the reconciliation loop issues zero PATCH calls at baseline).
  2. Phase 2 produces the Track 1 transitions required by README.md.

Usage:
    python scripts/verify_assignment.py

Exits 0 if every check passes, 1 otherwise.
"""
import sys
from pathlib import Path

import numpy as np

# Allow execution as `python scripts/verify_assignment.py` from the repository root.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.telemetry_bus import AGENTS, ROLES, get_baseline_state  # noqa: E402
from core.ecargo_gra_solver import solve_gra  # noqa: E402
from scenarios.group_scenario import (  # noqa: E402
    SCENARIO_NAME,
    compute_q_and_bounds,
    inject_scenario_event,
)

# Declarative starting labels from manifests/swarm_fleet.yaml
MANIFEST_LABELS = {
    "robot-a0": "lead-scout",
    "robot-a1": "precision-mapper",
    "robot-a2": "cargo-carrier",
    "robot-a3": "standby",
}

# Track 1 expected post-event assignment (README.md deliverables checklist)
EXPECTED_PHASE2 = {
    "robot-a0": "charging",
    "robot-a1": "precision-mapper",
    "robot-a2": "cargo-carrier",
    "robot-a3": "lead-scout",
}


def run_phase(title, telemetry, pending_jobs):
    """Compute Q/L, solve the ILP, print the matrix, and return {agent: role}."""
    Q, L_min, L_max = compute_q_and_bounds(telemetry, pending_jobs)
    X = solve_gra(Q, L_min, L_max)

    print(f"\n=== {title} ===")
    print(f"pending_jobs={pending_jobs}  L_min={L_min}  L_max={L_max}")
    print("Qualification matrix Q (rows = agents, cols = " + ", ".join(ROLES) + "):")
    for i, agent in enumerate(AGENTS):
        print(f"  {agent}: " + "  ".join(f"{v:5.3f}" for v in Q[i]))

    assignment = {}
    for i, agent in enumerate(AGENTS):
        j = int(np.argmax(X[i]))
        assignment[agent] = ROLES[j]
        t = telemetry[agent]
        print(
            f"  -> {agent}: {ROLES[j]:17s} Q={Q[i, j]:.3f} "
            f"(bat={t.battery:.0f}% lidar={'OK' if t.lidar_ok else 'FAIL'} "
            f"rssi={t.rssi_dbm:.0f}dBm dist={t.distance_m:.0f}m)"
        )
    print(f"Total swarm utility = {float(np.sum(Q * X)):.3f}")
    return assignment


def main() -> int:
    print(f"SCENARIO_NAME = {SCENARIO_NAME}")

    base_telemetry, base_jobs = get_baseline_state()
    phase1 = run_phase("Phase 1: Nominal Swarm Baseline", base_telemetry, base_jobs)

    event_telemetry, event_jobs = inject_scenario_event()
    phase2 = run_phase(f"Phase 2: {SCENARIO_NAME}", event_telemetry, event_jobs)

    ok = True

    print("\n=== Transition check ===")
    for agent in AGENTS:
        expected = EXPECTED_PHASE2[agent]
        arrow = "->" if phase1[agent] != phase2[agent] else "=="
        passed = phase2[agent] == expected
        ok &= passed
        status = "PASS" if passed else f"FAIL (expected {expected})"
        print(f"  {agent}: {phase1[agent]:17s} {arrow} {phase2[agent]:17s} [{status}]")

    print("\n=== Phase 1 equilibrium matches manifest labels ===")
    for agent in AGENTS:
        passed = phase1[agent] == MANIFEST_LABELS[agent]
        ok &= passed
        status = "PASS" if passed else f"FAIL (manifest = {MANIFEST_LABELS[agent]})"
        print(f"  {agent}: {phase1[agent]:17s} [{status}]")

    print("\nRESULT:", "ALL CHECKS PASSED" if ok else "CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
