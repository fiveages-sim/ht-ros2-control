# State-query evidence and scope (2026-11-11)

Current upstream main is 93ed844f7a183755c551d1363593a6d2d46b234a. Its IoLoop::pollOnce() calls send_get_motor_state_cmd(), whose SDK path sends the query internally, then calls motor_send_cmd() again on the same buffer.

## Historical A/B comparison (2026-10-30; no hardware rerun for this PR)

A is unchanged upstream; B only removes the redundant send in pollOnce. Each table uses exactly the first 5.000 s of its HOLD window. Both recorded 2500 MIT frames (500 Hz); active queries were A=4998 (999.60 Hz) vs B=2500 (500 Hz). A's two fewer boundary frames are window truncation, not evidence of loss.

| Motor | A distinct USB arrival Hz | B distinct USB arrival Hz | A longest visible silence incl. edges ms | B ms |
|---|---:|---:|---:|---:|
| J1 | 500.20 | 500.00 | 2.148 | 2.166 |
| J2 | 500.20 | 500.00 | 2.148 | 2.166 |
| J3 | 499.20 | 500.00 | 4.044 | 2.166 |
| J4 | 10.80 | 499.80 | 432.025 | 4.000 |
| J5 | 10.60 | 167.20 | 568.053 | 46.022 |
| J6 | 0.80 | 18.80 | 2329.216 | 347.998 |
| Gripper | 0.20 | 6.20 | 3266.872 | 1412.232 |

USB capture kernel-drop count was zero; MIT USB-completion maximum was 76 us in both A and B. USB arrival is not motor-internal sampling, and completion is not execution. The protocol lacks per-command sequence/ack association, so CAN/device packet-loss rate and command-response latency cannot be inferred.

A run: `20261030T075346Z-fiveages-MS-7E06-d2cec33ecd`.
B run: `20261030T075358Z-fiveages-MS-7E06-7e210f5575`.
Historical driver base: 93ed844; B code-only fix: 986df1a894883bd78805264a3d50d66b76683d3a.

This minimal correction improves query load but does not fix all uneven feedback: J6 and gripper still had long silence. The A/B sample is a single paired comparison, not randomized or long-duration validation. No control gains, packet encoding, shutdown-home behavior, SDK lifetime/reconnect or scheduling change is needed for the minimal pollOnce fix. Similar shutdown-home priming calls are intentionally outside this high-frequency, historically measured change.

The broader asynchronous transport proposal is separate and must not be attributed to this one-line correction. The separate minimal PR adds a hardware-free source-body/counting-sink regression; the async proposal has its own snapshot/dispatch and serialization tests. Neither test activates hardware or simulates the full serial/controller loop.
