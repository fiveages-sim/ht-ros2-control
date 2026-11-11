# Minimal query correction: no-device results (2026-11-11)

Upstream base: 93ed844f7a183755c551d1363593a6d2d46b234a; tested code commit: 30eea4bc9fe8a5b33ca0035dc80e74aac60de0e6. ROS 2 Jazzy; driver and SDK compiled, no hardware instantiated. Containers had no network/device passthrough, all capabilities dropped. Only the query-count CTest was selected; this is not a claim that every inherited lint suite passed.

| 500 synthetic cycles, one CAN port | Baseline | Minimal fix |
|---|---:|---:|
| MIT sends | 500 | 500 |
| Active query sends | 1000 | 500 |

The source-body/counting-sink oracle compiles actual SDK query/send bodies and pollOnce's call prefix. It does not run a real robot/serial object or the complete IO loop. New driver/SDK build and `ctest -R '^query_send_oracle$'` pass. The expected baseline count is specified explicitly rather than treating its duplicate count as a test infrastructure failure.

The unchanged baseline source bodies produce 1000 requests; the patched ones produce 500. This is packet-construction/call-count evidence, not measured Hz or firmware behavior. Historical paired real-HOLD data and its limitations are in [query-evidence.md](query-evidence.md).

- Baseline oracle run: `20261111T030948Z-fiveages-MS-7E06-940622f2de`.
- Fixed oracle run: `20261111T030949Z-fiveages-MS-7E06-edc98a4fc2`.
- Build run: `20261111T030950Z-fiveages-MS-7E06-50851fa0a0`.
- CTest run: `20261111T030956Z-fiveages-MS-7E06-7e3df8042f`.
- Locked image: `sha256:56ac7597b42f9583b2257aba175129fd12b46c9acc95f4548c8ad142551c14eb`.

No controller, motor motion, gripper task, enable or release operation was run for this PR. Similar shutdown-home priming calls and the broader async scheduling/lifecycle changes are not modified here.
