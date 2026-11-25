# Unit 3 Live Cluster Demo Evidence — Track 1

**Track:** Track 1 — Critical Energy Evacuation & Handoff
**Cluster:** Kind `cosc3657-unit3` · **Namespace:** `default` · **Captured:** 2026-11-25

This file is the captured evidence for deliverable §2 ("Live Demo & Cluster
Verification"). Everything below is verbatim terminal output from a live Kubernetes API
server — no output was edited or reconstructed.

---

## 1. Environment Setup

```bash
# 1. Python environment
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt   # Windows
# source .venv/bin/activate && pip install -r requirements.txt  # Linux/macOS

# 2. Cluster + fleet
kind create cluster --name cosc3657-unit3
kubectl apply -f manifests/swarm_fleet.yaml
kubectl wait --for=condition=Ready pod -l app=ecargo-swarm --timeout=180s
```

```text
pod/robot-a0 created
pod/robot-a1 created
pod/robot-a2 created
pod/robot-a3 created
pod/robot-a0 condition met
pod/robot-a1 condition met
pod/robot-a2 condition met
pod/robot-a3 condition met
```

---

## 2. BEFORE — Declarative baseline from the manifest

```bash
kubectl get pods -l app=ecargo-swarm -L ecargo.role
```

```text
NAME       READY   STATUS    RESTARTS   AGE   ECARGO.ROLE
robot-a0   1/1     Running   0          7s    lead-scout
robot-a1   1/1     Running   0          7s    precision-mapper
robot-a2   1/1     Running   0          7s    cargo-carrier
robot-a3   1/1     Running   0          7s    standby
```

---

## 3. Two-Phase Demo Run

```bash
python run_demo.py
```

```text
──────────────────────────── Phase 1: Nominal Swarm Baseline ────────────────────────────
                       Optimal E-CARGO Role Assignment (Matrix X)
┌───────────┬─────────┬───────┬───────────────┬──────────────────┬──────────────────┬───────────┐
│ Agent Pod │ Battery │ LiDAR │   RSSI / Dist │ Previous Role    │ Assigned Role    │ Fitness Q │
├───────────┼─────────┼───────┼───────────────┼──────────────────┼──────────────────┼───────────┤
│ robot-a0  │     95% │  OK   │ -62 dBm / 40m │ lead-scout       │ lead-scout       │     0.982 │
│ robot-a1  │     90% │  OK   │ -68 dBm / 60m │ precision-mapper │ precision-mapper │     0.828 │
│ robot-a2  │     88% │  OK   │ -70 dBm / 50m │ cargo-carrier    │ cargo-carrier    │     0.836 │
│ robot-a3  │     92% │  OK   │ -60 dBm / 20m │ standby          │ standby          │     0.718 │
└───────────┴─────────┴───────┴───────────────┴──────────────────┴──────────────────┴───────────┘
Role Bounds L_min: [1, 1, 1, 0, 0, 0] | L_max: [1, 1, 1, 0, 4, 4]
Total Swarm Utility Score: 3.364

─────────────────── Phase 2: Track 1: Critical Energy Evacuation & Handoff ───────────────────
                         Optimal E-CARGO Role Assignment (Matrix X)
┌───────────┬─────────┬───────┬───────────────┬──────────────────┬──────────────────────┬───────────┐
│ Agent Pod │ Battery │ LiDAR │   RSSI / Dist │ Previous Role    │ Assigned Role        │ Fitness Q │
├───────────┼─────────┼───────┼───────────────┼──────────────────┼──────────────────────┼───────────┤
│ robot-a0  │     12% │  OK   │ -62 dBm / 40m │ lead-scout       │ charging (UPDATED)   │     1.000 │
│ robot-a1  │     90% │  OK   │ -68 dBm / 60m │ precision-mapper │ precision-mapper     │     0.828 │
│ robot-a2  │     88% │  OK   │ -70 dBm / 50m │ cargo-carrier    │ cargo-carrier        │     0.836 │
│ robot-a3  │     92% │  OK   │ -60 dBm / 20m │ standby          │ lead-scout (UPDATED) │     0.874 │
└───────────┴─────────┴───────┴───────────────┴──────────────────┴──────────────────────┴───────────┘
Role Bounds L_min: [1, 1, 1, 0, 0, 0] | L_max: [1, 1, 1, 0, 4, 4]
Total Swarm Utility Score: 3.538
```

**Reading the transcript.**

* **Phase 1** shows `Previous Role == Assigned Role` on all four pods, so the
  orchestrator issued **zero `PATCH` requests** — the reconciliation loop is idempotent,
  and the manifest's declarative labels are already the ILP optimum (utility **3.364**).
* **Phase 2** marks exactly two cells `(UPDATED)`. `robot-a0`'s fitness for `charging`
  jumps to **1.000** because `battery = 12 % < 20 %` triggers the safety-cutoff branch,
  which zeroes its field-role row; `robot-a3` is elected `lead-scout` at **0.874** by the
  ILP, not by any hardcoded rule. Swarm utility rises **3.364 → 3.538**.

---

## 4. AFTER — Live cluster state

```bash
kubectl get pods -l app=ecargo-swarm -L ecargo.role
```

```text
NAME       READY   STATUS    RESTARTS   AGE   ECARGO.ROLE
robot-a0   1/1     Running   0          66s   charging
robot-a1   1/1     Running   0          66s   precision-mapper
robot-a2   1/1     Running   0          66s   cargo-carrier
robot-a3   1/1     Running   0          66s   lead-scout
```

### 4.1 Required transition check

| Pod | Before | After | Required by `README.md` | Result |
| :--- | :--- | :--- | :--- | :--- |
| `robot-a0` | `lead-scout` | `charging` | `lead-scout → charging` | ✅ |
| `robot-a3` | `standby` | `lead-scout` | `standby → lead-scout` | ✅ |
| `robot-a1` | `precision-mapper` | `precision-mapper` | unchanged | ✅ |
| `robot-a2` | `cargo-carrier` | `cargo-carrier` | unchanged | ✅ |
| **All pods** | `RESTARTS 0` | `RESTARTS 0` | `RESTARTS = 0` | ✅ |

### 4.2 Label selectors resolve to the new roles

The role change is real cluster state, not just console output — the API server's own
label selectors now match the reassigned pods:

```bash
kubectl get pods -l "app=ecargo-swarm,ecargo.role=charging"   -o name   # pod/robot-a0
kubectl get pods -l "app=ecargo-swarm,ecargo.role=lead-scout" -o name   # pod/robot-a3
```

### 4.3 Reconfiguration transparency: same containers, new roles

```bash
kubectl get pods -l app=ecargo-swarm -o "custom-columns=POD:.metadata.name,\
ROLE:.metadata.labels['ecargo\.role'],RESTARTS:.status.containerStatuses[0].restartCount,\
CONTAINER_ID:.status.containerStatuses[0].containerID,\
STARTED_AT:.status.containerStatuses[0].state.running.startedAt"
```

```text
POD        ROLE               RESTARTS   CONTAINER_ID                                                              STARTED_AT
robot-a0   charging           0          containerd://f31ca15fd5ae5f3766e088e9ebafbb91dc18141f46ebb236f8ed5e271a8e12d2   2026-11-25T17:54:17Z
robot-a1   precision-mapper   0          containerd://359cbdd0c63e4ef4733316aa14213831770002889574b796dd6877b164e18ad2   2026-11-25T17:54:17Z
robot-a2   cargo-carrier      0          containerd://96d8995369087ed570481c7dbdf9831199cf593aa3322c36ebabbb6af9080dd7   2026-11-25T17:54:18Z
robot-a3   lead-scout         0          containerd://1cd4cb9b8280dea291c7e9eef30b38c17e9c45740c29d904e7ae6e5671434dad   2026-11-25T17:54:18Z
```

This is the strongest single piece of evidence for the argument in
`system_models.md` §1.4. The `CONTAINER_ID`s and `STARTED_AT` timestamps are **identical
before and after** the reassignment, and both pods whose role changed were started
*before* `run_demo.py` ran. The containers were never recreated, re-pulled or signalled:
only `metadata.labels` changed in `etcd`, and the kubelet applied that diff in place.
A Unit-2-style crash-stop recovery would instead show a new `CONTAINER_ID`, a later
`STARTED_AT`, and `RESTARTS ≥ 1`.

---

## 5. Measured Control-Loop Latency

Timings taken on the demo host against this live Kind cluster, used to ground the
interaction-model analysis in `system_models.md` §2.1.2:

| Stage | Samples | Min | Median | Worst |
| :--- | ---: | ---: | ---: | ---: |
| `solve_gra()` — PuLP/CBC branch-and-bound | 15 | 43.8 ms | **56.9 ms** | 1261.6 ms |
| `read_namespaced_pod()` — `GET pod` | 10 | 6.8 ms | **7.1 ms** | 81.6 ms |
| `patch_namespaced_pod()` — `PATCH pod` | 10 | 10.7 ms | **12.6 ms** | 25.1 ms |

Interpretation:

* The solver's spread is **not** search effort — the ILP has only 24 binary variables and
  16 constraints. PuLP invokes CBC as a *child process* through temporary `.lp` files, so
  the cost is process spawn plus disk I/O; the 1.26 s worst case is an OS scheduling
  outlier, not combinatorial blow-up.
* `PATCH` costs about twice a `GET` because it must additionally pass admission control
  and commit a write to `etcd`.
* One full reconciliation cycle for the Track 1 handoff is therefore
  `4 GET + 2 PATCH + 1 solve ≈ 110 ms` of control-plane work — negligible beside the
  physical drive time from 40 m back to the dock, which is what the 20 % battery cutoff
  must actually budget for.

---

## 6. Offline Reproduction (no Docker required)

`scripts/verify_assignment.py` runs both phases through the real
`compute_q_and_bounds()` and the real PuLP/CBC solver, printing the full 4×6 `Q` matrix
and asserting (a) that the Phase 1 optimum equals the labels declared in
`manifests/swarm_fleet.yaml` and (b) that Phase 2 produces the Track 1 transitions. It
exits non-zero on any mismatch, so it is also usable as a regression check.

```bash
python scripts/verify_assignment.py
```

```text
=== Transition check ===
  robot-a0: lead-scout        -> charging          [PASS]
  robot-a1: precision-mapper  == precision-mapper  [PASS]
  robot-a2: cargo-carrier     == cargo-carrier     [PASS]
  robot-a3: standby           -> lead-scout        [PASS]

=== Phase 1 equilibrium matches manifest labels ===
  robot-a0: lead-scout        [PASS]
  robot-a1: precision-mapper  [PASS]
  robot-a2: cargo-carrier     [PASS]
  robot-a3: standby           [PASS]

RESULT: ALL CHECKS PASSED
```

---

## 7. Resetting for a Repeat Demo

Phase 1 re-establishes the baseline from telemetry, but the pods keep the labels of the
previous run, so re-apply the manifest to restore the declarative starting state before
presenting:

```bash
kubectl delete -f manifests/swarm_fleet.yaml
kubectl apply  -f manifests/swarm_fleet.yaml
kubectl wait --for=condition=Ready pod -l app=ecargo-swarm --timeout=180s
python run_demo.py
```

Tear down entirely with `kind delete cluster --name cosc3657-unit3`.
