"""
Group Scenario Implementation Module -- Track 1: Critical Energy Evacuation & Handoff.

Scenario
--------
The forward exploration agent (`robot-a0`, baseline role `lead-scout`) suffers energy
exhaustion: its traction battery falls to 12.0%, below the 20.0% mission safety cutoff.
The E-CARGO control plane must preemptively evacuate it to the `charging` role and
re-elect a `lead-scout` from the remaining fleet -- without restarting any container and
without any agent-to-role pair being hardcoded.

Expected Kubernetes transitions (Phase 1 -> Phase 2)
----------------------------------------------------
    robot-a0 : lead-scout       -> charging
    robot-a1 : precision-mapper -> precision-mapper  (unchanged)
    robot-a2 : cargo-carrier    -> cargo-carrier     (unchanged)
    robot-a3 : standby          -> lead-scout

All transitions are emergent properties of the ILP objective
`maximize sum(Q[i,j] * X[i,j])` under the role-cardinality bounds [L_min, L_max];
see `docs/system_models.md` for the derivation and the CDK5 Ch. 2 models.
"""
import math
from typing import Dict, List, Tuple
import numpy as np
from core.telemetry_bus import AGENTS, ROLES, AgentTelemetry, get_baseline_state

SCENARIO_NAME = "Track 1: Critical Energy Evacuation & Handoff"
CRITICAL_BATTERY_THRESHOLD = 20.0
RSSI_PARTITION_THRESHOLD = -85.0

def inject_scenario_event() -> Tuple[Dict[str, AgentTelemetry], int]:
    """
    Inject this group's Track 1 disruption: critical energy depletion on robot-a0.

    Starts from the nominal baseline snapshot and mutates exactly one raw hardware
    metric -- robot-a0's battery, driven from 95.0% to 12.0%, i.e. below the 20.0%
    CRITICAL_BATTERY_THRESHOLD safety cutoff. Every role change observed in Phase 2 is
    then derived by compute_q_and_bounds() and the GRA solver, not scripted here.

    Returns:
        (telemetry, pending_cargo_jobs) -- the disrupted environment state E.
    """
    telemetry, pending_jobs = get_baseline_state()

    # --- Track 1 Implementation: Critical Energy Evacuation & Handoff -------------
    # Environmental event: the forward `lead-scout` (robot-a0) has been operating at
    # 40 m from the base gateway long enough to drain its traction battery below the
    # CRITICAL_BATTERY_THRESHOLD (20.0%) safety cutoff. This is a *resource
    # exhaustion* / partial-degradation fault (CDK5 2.4.2), not a crash-stop fault:
    # the pod stays Running and keeps publishing telemetry, so the control plane can
    # observe the degradation and hand the role off *before* the agent strands itself.
    #
    # Only the raw hardware metric is mutated here -- no agent-to-role assignment is
    # hardcoded. The battery value flows into compute_q_and_bounds(), which zeroes
    # robot-a0's field-role qualifications and raises Q[a0, charging] to 1.0, and the
    # ILP solver independently elects the best remaining lead-scout.
    telemetry["robot-a0"].battery = 12.0

    # The mission's cargo backlog is unchanged by an energy event; the job queue stays
    # at its nominal depth so that L_min[cargo-carrier] remains 1 (contrast Track 4).
    return telemetry, pending_jobs


def compute_q_and_bounds(
    telemetry: Dict[str, AgentTelemetry],
    pending_jobs: int,
) -> Tuple[np.ndarray, List[int], List[int]]:
    """
    Compute the 4x6 Qualification Matrix Q and role capacity vectors (L_min, L_max).
    Must produce valid optimal assignments for both Phase 1 (Baseline) and Phase 2 (Disrupted).

    Roles index order (ROLES):
      0: lead-scout
      1: precision-mapper
      2: cargo-carrier
      3: signal-relay
      4: charging
      5: standby
    """
    m, n = len(AGENTS), len(ROLES)
    Q = np.zeros((m, n))

    min_rssi = min(t.rssi_dbm for t in telemetry.values())
    max_dist = max(t.distance_m for t in telemetry.values())
    relay_needed = 1 if min_rssi < RSSI_PARTITION_THRESHOLD else 0
    midpoint_dist = max_dist / 2.0

    for i, name in enumerate(AGENTS):
        t = telemetry[name]
        bat = t.battery / 100.0

        # Preemptive energy safety cutoff (< 20%): force charging role, zero out field roles
        if t.battery < CRITICAL_BATTERY_THRESHOLD:
            Q[i, ROLES.index("charging")] = 1.00
            continue

        # r0: lead-scout (high battery + healthy LiDAR + forward range readiness)
        scout_pos = 0.08 if t.distance_m >= 40.0 else 0.0
        Q[i, ROLES.index("lead-scout")] = round(
            min(1.0, bat * (0.95 if t.lidar_ok else 0.05) + scout_pos), 3
        )

        # r1: precision-mapper (requires healthy LiDAR; optimal in mid-field 35-80m)
        field_factor = 1.0 if 35.0 <= t.distance_m <= 80.0 else 0.80
        Q[i, ROLES.index("precision-mapper")] = round(
            bat * 0.92 * field_factor if t.lidar_ok else 0.02, 3
        )

        # r2: cargo-carrier (battery * payload_cap)
        Q[i, ROLES.index("cargo-carrier")] = round(bat * t.payload_cap, 3)

        # r3: signal-relay (spatial midpoint affinity when relay is needed)
        if relay_needed and max_dist > 0:
            spatial_fit = max(0.0, 1.0 - abs(t.distance_m - midpoint_dist) / max_dist)
            Q[i, ROLES.index("signal-relay")] = round(spatial_fit, 3)
        else:
            Q[i, ROLES.index("signal-relay")] = 0.05

        # r4: charging (low affinity when battery >= 20%)
        Q[i, ROLES.index("charging")] = 0.05

        # r5: standby (energy-conserving dock reserve affinity near base)
        dock_bonus = 0.35 if t.distance_m <= 25.0 else 0.0
        Q[i, ROLES.index("standby")] = round(bat * 0.40 + dock_bonus, 3)

    required_carriers = min(2, max(1, math.ceil(pending_jobs / 2)))
    L_min = [1, 1, required_carriers, relay_needed, 0, 0]
    L_max = [1, 1, required_carriers, relay_needed, m, m]

    return Q, L_min, L_max
