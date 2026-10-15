# E3 pilot on 48813ac — free-run analysis

**Measured by the user** on 2026-10-15 (five runs, STK graphics off);
**derived from the run logs** with `src/acav/scripts/analyze_coupling.py`
(commit `1b561a5`) and the analysis behind it. Run folders:
`results/E6_E3pilot_48813ac` and `results/E3_pilot_48813ac`.

This is a pilot, not an E3 result: two free-run runs at rtf 1.0 on code
that has since changed (free-run pacing `a6436a4`; free-run provenance
`9cbc0f7`). It explains why free-run degraded and what was fixed.

## Runs

| Run | Mode | Captured | Signature = E6-smoke-1 | Commands | Control rate | Gap p50 / p90 / max (s) | Gaps > 0.6 s (total) | max n | COM stalls > 300 ms |
|---|---|---|---|---|---|---|---|---|---|
| E6-E3P-1 | lockstep | 5/5 | yes | 7199 | 10.00 Hz | 0.100 / 0.100 / 0.100 | 0 | 1 | 3 |
| E3 lockstep seed0 | lockstep | 5/5 | yes | 7199 | 10.00 Hz | 0.100 / 0.100 / 0.100 | 0 | 1 | 5 |
| E3 lockstep seed1 | lockstep | 5/5 | yes | 7199 | 10.00 Hz | 0.100 / 0.100 / 0.100 | 0 | 1 | 1 |
| E3 free-run seed0 | free_run | 0/5 | no | 2983 | 4.14 Hz | 0.219 / 0.281 / 3.688 | 14 (15.8 s) | 37 | 12 |
| E3 free-run seed1 | free_run | 1/5 | no | 2542 | 3.53 Hz | 0.235 / 0.359 / 5.672 | 73 (85.7 s) | 56 | 63 |

All five ran with graphics off and rtf 1.0. The lockstep runs have
3 saturations, the free-run runs 10 and 14; no run entered SAFE. The
free-run start gate held the clock at t = 0 for 50.8 s and 434.7 s;
the second wait was a 6 min 20 s pause of the run orchestrator between
two consecutive log lines (likely a Windows console QuickEdit
selection; not proven), during which the gate kept the window intact.
The first command was stamped 0.100 s in every run.

Free-run errors at the image epochs were 0.27–1.14° (lockstep 0.002°,
T3 0.107°); every free-run failure is `not_settled`.

## What the logs show

1. **Free-run cycles cost their work plus about 0.1 s.** The rate groups
   mark their run after sensing and the loop slept until that mark plus
   the fast period: 239 / 260 ms cycles for about 134 / 173 ms of work.
   Fixed in `a6436a4` (fixed-rate schedule from the cycle start).
2. **Sensing does four queries per cycle in free-run, against about one
   in lockstep.** Combined attitude/rate (B) can verify only at an epoch
   where the bridge wrote a sample, which a free-run sense epoch never
   is, so it stayed "pending"; orbit batching (C) is lockstep-only.
   Sense 117.7 / 148.1 ms against 37 ms. Since `9cbc0f7` free-run runs
   record B off and C at 1. C stays off in free-run, because reading
   epochs ahead of the wall clock is invalid in an asynchronous loop.
3. **Single STK COM calls take seconds.** At the long gaps the bridge
   itself published nothing, and the COM round trip reached 3562 ms
   (seed0) and 2234 ms (seed1). The cause is not in the logs (see the
   backlog: the twin's Simulink loop load is a candidate). A few gaps
   in seed1 (t ≈ 386–415 s) were on the twin side.
4. **STK's read-back is not the twin's attitude in free-run.** Every
   free-run read-back is at an epoch after the newest written sample
   (2984 of 3011 and 2555 of 2771), where STK reports an attitude a
   median 20.6° / 19.7° (max 61°) away; lockstep reads back at written
   stamps with 0.0000° difference. This is a finding of the study:
   asynchronous coupling makes STK report an attitude the twin never
   had. The twin feeds back its own state, not the read-back.
5. **The twin's error is not explained by stale samples alone.** The
   newest sample at each image epoch is 0.016–0.312 s old, and the
   twin's own rate (median 0.33°/s) times that age is at most about
   0.1°. Hypothesis: the feed-forward rate in `twin_step.m:119-131`
   divides the line-of-sight change of a whole sample interval by one
   0.1 s sub-step, and the later sub-steps get zero, which pulses the
   rate reference. To be checked by an offline replay before any twin
   change.
6. **Command-stamp offset.** 2878 of 2983 and 2468 of 2542 free-run
   commands are stamped later than the state they carry, by up to
   0.097 s (mean 0.048 s); every free-run command is written behind
   the bridge clock (write lag median 0.119 / 0.135 s). At the twin's
   rate that moves the written attitude by about 0.02–0.12°, small
   against the 2° sensor cone and the 0.5° capture tolerance; it is
   recorded, and the stamping is not changed.

## Decisions taken after the pilot

- Fixes are separate gated items; lockstep must stay bit-identical
  (same signature as E6-smoke-1).
- No `SetAttitude RealTime`: the read-back finding is reported, not
  hidden.
- The real-time factor is a recorded, enforced run condition
  (`9cbc0f7`); E3 arms use rtf 1.0 and 0.5.
- C stays off in free-run, with no extra arm.
- E4 is removed from the paper; E5 becomes E3's seeded fault replay.
