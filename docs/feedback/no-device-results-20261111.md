# Async transport: no-device results (2026-11-11)

Upstream base: 93ed844f7a183755c551d1363593a6d2d46b234a; tested code commit: 5d63824d20f2a4503b084948bf0362066797c7ba. Both actual SDK and driver compiled. Containers had no network/device passthrough, all capabilities dropped. Only the two named no-device CTests were selected; this is not a complete lint/workspace or end-to-end hardware validation.

| Check | Result |
|---|---|
| Synthetic decoder updates | 10000 |
| Concurrent snapshots checked in this run | 5993 |
| Coherency / monotonic count/time errors | 0 |
| Command-only dispatch decisions | 500, no appended query |
| Idle-only query decisions | 10 |
| Extra query after throwing send | 0 |
| MIT frames / motor references | 128 / 896 |
| Encoding stream size | 9856 bytes |
| Baseline vs async stream | Exactly equal |

Both serialization streams SHA256: `4e9a404631816d8175ea707f358061e231e6721ba35be9cc14d9261f8440fc4f`. These test executables instantiate only in-memory motor objects, never robot, serial_driver or hardware plugins. Source-body baseline serialization is compiled from the unchanged SDK. No throughput/frequency, hot-plug, object-rebuild, destructor release, complete thread wakeup or device stopping guarantee is inferred.

- Build run: `20261111T030956Z-fiveages-MS-7E06-5e2b277c7b`.
- Snapshot/dispatch and serialization CTests: `20261111T031003Z-fiveages-MS-7E06-1a687a29cd`.
- Baseline serialization run: `20261111T031003Z-fiveages-MS-7E06-f22461fcaf`.
- Locked image: `sha256:56ac7597b42f9583b2257aba175129fd12b46c9acc95f4548c8ad142551c14eb`.

The PR remains a draft because it also changes serial-error and lifecycle policy. Historical A/B/C/D real-HOLD results are separately labeled in [async-proposal.md](async-proposal.md), not presented as new hardware tests of this PR.
