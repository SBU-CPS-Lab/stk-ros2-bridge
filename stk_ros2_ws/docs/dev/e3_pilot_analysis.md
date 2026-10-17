# E3 pilot on 48813ac — free-run analysis

> **Index.** Engineering history, not current documentation. The analysis
> of the first E3 free-run pilot (48813ac, 2026-10-15): why free-run
> degraded (unpaced loop, held geometry, the clock published before
> sensing) and the fixes that followed. Its run folders are now under
> results/_archive/freerun_fixes. The current free-run design is in
> [../05_time_synchronization.md](../05_time_synchronization.md); current
> results in [../09_live_runs_and_results.md](../09_live_runs_and_results.md).

**Measured by the user** on 2026-10-15 (five runs, STK graphics off);
**derived from the run logs** with `src/acav/scripts/analyze_coupling.py`
(commit `1b561a5`) and the analysis behind it. Run folders:
`results/_archive/freerun_fixes/E6_E3pilot_48813ac` and `results/_archive/freerun_fixes/E3_pilot_48813ac`.

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
   rate reference. **Confirmed by the offline replay below, and fixed
   in `72be372`** (see "Twin geometry").
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

## The fixes, and what they changed

Commits on `perf/e6-throughput`, each its own gated item. The live
numbers below are **measured by the user, single runs** (one per
condition and commit, STK graphics off, seed 0), and were derived from
the run logs with `analyze_coupling.py`; the CPU sampler was running in
each of them.

### Offline replay of the twin's free-run feed-forward (P2c)

`adcs.simulink_step` driven offline by the bridge sample times recorded
in the two pilot free-run runs (22 windows of 62.5 s around a T1-like
target on a synthetic circular orbit), with the unchanged
`adcs.twin_step`:

| Variant | Error at image, median / max (°) | Max error in hold, median / max (°) | Captured | Saturation episodes |
|---|---|---|---|---|
| a: as on 48813ac (geometry held over all sub-steps) | 0.763 / 1.095 | 0.797 / 1.253 | 1/22 | 58 |
| b: geometry extrapolated to each sub-step (newer sample, backwards) | 0.003 / 0.004 | 0.004 / 0.004 | 22/22 | 0 |
| b′: causal -- newest sample at or before the sub-step, propagated forward | 0.003 / 0.004 | 0.004 / 0.0042 | 22/22 | 0 |
| c: geometry held, rate over the true sample interval | 0.057 / 0.486 | 0.103 / 0.486 | 22/22 | 1 |
| lockstep cadence (reference) | 0.0029 | 0.0041 | -- | -- |

Every variant is bit-identical to the lockstep branch at lockstep
cadence. The cause is the geometry held across sub-steps; b′ was
implemented.

### Pacing (`a6436a4`)

Free-run cycles cost their work plus about 0.1 s (the loop slept until
the rate groups' mark after sensing plus the period): 239 / 260 ms
cycles in the pilot. Cycles now run on a fixed-rate schedule from the
cycle start; an overrun starts the next cycle at once and is counted.
After it, at rtf 1.0: 124 ms mean cycles on `72be372` (5802 cycles).

### Twin geometry (`72be372`)

Each free-run sub-step now uses the newest sample stamped at or before
its time, propagated forward to it (`adcs.geometry_at`: position along
the velocity, target rotated with the Earth; the Earth rate is defined
once, in `adcs.config`, and the targets must be Earth-fixed). At rtf 0.5
on `72be372` all five targets were captured with errors at the image
epoch of 0.001-0.11°, against 0.27-1.14° in the pilot's free-run runs.

### Record integrity (`168cf19`, `1816a36`)

At rtf 1.0 on `72be372` the twin published nothing between 247.734 and
252.219 s, so T4's hold window held no telemetry; the scheduler dropped
T4 from the record, which stayed reportable (3/4 instead of 3/5). An
unscorable target is now recorded (`no_telemetry`, `no_access_answer`;
the latter, and `no_telemetry` in lockstep, make the record not
reportable), and `check_record` and `report` refuse a record whose
targets are not exactly its scenario's. Of the existing records, only
that run is refused.

### Access coverage (`88e60f1`)

At rtf 1.0 on `72be372`, T2 scored `no_access` with a 0.0018° pointing
error: access was computed while the newest written attitude was
stamped 109.959 s, before the 110.0 s image epoch, and beyond its
newest sample STK reports an attitude near nadir. The bridge now
answers a ComputeAccess whose stop time the written attitude does not
reach yet with `ATTITUDE_NOT_COVERED`, the scheduler asks again, and
the wait is recorded per target; a target never covered is
`access_unverified` (not reportable). Lockstep is answered at once. At
rtf 1.0 on `4b26de0`, T4 waited 0.672 s and T6 0.094 s, and all five
were captured.

### Twin pacing (`4b26de0`)

Unpaced, the Simulink loop stepped as fast as it could and MATLAB used
about one core in every mode. The model now steps at a requested
100 Hz (Simulink pacing, recorded in the twin identity
`v2 ... 100Hz`); the identity is published once a second. MATLAB's CPU
in E6 fell from 1.04 to 0.62 cores (user's sampling window). At rtf 1.0
the twin then answered fewer samples (consumed share 0.439 -> 0.314),
which led to the next fix.

### Clock order (`1628866`, with the loop counters `8345551`)

The twin steps only when its attitude, orbit and rate stamps equal the
latest ScenarioTime, and free-run published ScenarioTime before
sensing: for a whole sensing time per cycle the twin could not step, so
it could answer a sample only between the end of sensing and the next
cycle. ScenarioTime is now published after sensing with the same
`now`. The twin also logs its free-run loop counters
(`<run>.logs/twin_loop.csv`): within the window it made 73.4 (rtf 1.0)
and 73.7 (rtf 0.5) calls per wall second, 99 % of them ready.

### Live results

| Commit | Run | Captured | Saturations | Control rate | Consumed share | Gap max (s) | Gaps > 0.6 s (total s) | max n | Bridge overruns | Read-back beyond newest, median |
|---|---|---|---|---|---|---|---|---|---|---|
| `72be372` | E6 lockstep | 5/5 | 3 | 10.00 Hz | 1.00 | 0.1 | 0 | 1 | -- | -- |
| `72be372` | free-run rtf 1.0 | 3 of 4 recorded (T4 dropped) | 4 | 3.54 Hz | 0.439 | 16.05 | 159 (219.8) | 160 | 4583/5802 | 20.9° |
| `72be372` | free-run rtf 0.5 | 5/5 | 3 | 8.80 Hz | 0.911 | 1.77 | 11 (11.1) | 17 | 478/6954 | 0.058° |
| `4b26de0` | E6 lockstep | 5/5 | 3 | 10.00 Hz | 1.00 | 0.1 | 0 | 1 | -- | -- |
| `4b26de0` | free-run rtf 1.0 | 5/5 | 2 | 2.81 Hz | 0.314 | 6.17 | 212 (175.2) | 61 | 3597/6455 | 20.1° |
| `4b26de0` | free-run rtf 0.5 | 5/5 | 3 | 9.07 Hz | 0.931 | 1.91 | 12 (11.6) | 19 | 348/7021 | 0.065° |
| `8345551` | E6 lockstep | 5/5 | 3 | 10.00 Hz | 1.00 | 0.1 | 0 | 1 | -- | -- |
| `8345551` | free-run rtf 1.0 | 5/5 | 3 | 7.27 Hz | 0.991 | 3.44 | 7 (8.7) | 34 | 5194/5279 | 21.5° |
| `8345551` | free-run rtf 0.5 | 5/5 | 3 | 9.51 Hz | 0.982 | 1.12 | 4 (3.5) | 11 | 499/6979 | 0.065° |

Every E6 lockstep run reproduced the E6-smoke-1 determinism signature.
E6 step time: 76.4 ms (`72be372`), 60.7 ms (`4b26de0`), 79.1 ms
(`8345551`) with no lockstep code change between the last two; the
`8345551` session used a CPU sampler that queries every process through
WMI each second, which is suspected, not shown, to cost the difference.
E6 timing for the paper is therefore measured without any CPU sampler,
as mean ± sd over repeated runs in one session.

At rtf 1.0 on `8345551` the twin answers 99 % of the bridge's samples;
the limit is now the bridge itself (sense about 124 ms against the
0.1 s period, 98 % of cycles overrunning), a genuine host and COM cost
with the combined read (B) and orbit batching (C) off in free-run.

## E3 protocol (for the campaign)

- **Arms:** lockstep; free-run at rtf 1.0; free-run at rtf 0.5. Seeds
  and repeats as decided for the campaign.
- **Report per run:** captures and outcome per target, determinism
  signature, control rate, consumed share, command-gap distribution
  (p50, p90, max, count and total over 0.6 s), max n, STK read-back
  deviation beyond the newest written sample, bridge overruns, twin
  calls per wall second (`twin_loop.csv`), access waits.
- **CPU sampling only on diagnostic runs**, never on runs whose timing
  is reported.
