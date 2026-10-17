# Frames and Units

## Units: SI, forced at the STK end

Backends call `configure_units()` during `connect()`, setting STK's
`UnitPreferences` to metres, seconds, radians, kilograms. After that,
every data provider and every raw Connect report returns SI natively --
there is **no unit conversion inside the bridge** for distance, time,
mass, or (almost) angle. See `backends/base.py`'s "Units contract" for
why this lives at the STK boundary rather than in `core/conversions.py`.

### The one exception: angular rate

Confirmed on a live STK 12.6 install: `SetCurrentUnit("AngleRate",
"rad/sec")` is **rejected** ("Cannot set the specified unit."). The
bridge does not guess the resulting unit -- `core/validation.py`
**measures** it at startup by commanding a known body rate and reading
it back, installing the result as `AngleRateScale` on the validation
report. `core/sensing.py` applies that measured scale to every
angular-rate field. If validation could not measure it, angular-rate
output is marked unverified in the logs (assumed rad/s) rather than
silently trusted.

## Quaternions: (x, y, z, w), confirmed

STK's Q1..Q3 is the vector part, Q4 the scalar part -- the same
convention as `geometry_msgs/Quaternion` and ROS generally. Confirmed
empirically on a live STK 12.6 install by `core/validation.py`'s
`quaternion_ordering` check: a deliberately asymmetric test rotation is
commanded and read back, and the round-trip error is checked. This is
not assumed from documentation alone -- see that check's docstring for
why a permutation and a frame error look similar (large error) but are
distinguishable (permutation preserves the multiset of component
magnitudes; a frame rotation does not), a distinction that mattered
during this project's actual development (see the note below).

## Frames: what STK 12.6 actually offers

Confirmed against a live inventory (`Cartesian Position` group members):

```
TrueOfDateRotating, Fixed, ICRF, MeanOfDate, MeanOfEpoch, TrueOfDate
```

**"J2000" is NOT one of these.** `core/frames.py`'s registry aliases
`j2000` to `MeanOfEpoch` rather than exposing it as a real frame, so
requests for `j2000` resolve to something STK 12.6 actually has, with
the caveat documented at the alias site (`MeanOfEpoch` matches J2000
only when the scenario epoch is itself J2000).

## Attitude command/read-back pairing -- the frame trap

STK has **two independent real-time attitude command forms**, each with
exactly one correct read-back provider. Pairing them wrong does not
error -- it silently returns an attitude differing by the Earth-rotation
angle at the sample epoch, which is large (tens of degrees) but entirely
plausible-looking as a number.

| Command | Read-back provider |
|---|---|
| `AddAttitude CBFQuat` (central-body-fixed) | `ECF Attitude Quaternions` |
| `AddAttitude Quat` (inertial) | `Attitude Quaternions` |

Both providers are **ungrouped** on STK 12.6 -- they take no reference-
frame argument, so passing one is silently ignored. This exact mismatch
(commanding CBFQuat, reading via the inertial provider) produced a
~99 degree "quaternion ordering" failure during this project's Phase 3
bring-up that was actually a frame-pairing bug, not an ordering bug --
see `core/validation.py`'s `_check_quaternion_ordering` docstring for
the full story and the diagnostic that now distinguishes the two cases
automatically.

`core/sensing.py` reads whichever provider `core/validation.py`
confirmed round-trips correctly (recorded as
`ValidationReport.attitude_provider`) -- it does not hardcode a choice.
On STK 12.6 both pairings round-trip; validation then prefers the one
that matches the configured command form
(`actuation.default_attitude_inertial`), and the bridge refuses to
start if the two still disagree (`nodes/bridge_node.py`
`check_attitude_pairing`). The ACAV configuration commands the inertial
form (`default_attitude_inertial: true` in
`src/acav/config/bridge_params_acav.yaml`), so it reads back through
`Attitude Quaternions`. Experiment E1 forces the mis-paired
`ECF Attitude Quaternions` on purpose
(`sensing.force_attitude_provider`, `acav/campaign.py`
`MISPAIRED_ATTITUDE_PROVIDER`); the provider each run actually used is
reported on BridgeStatus and recorded in the run record.

## Position command frames

`SetPosition` accepts: `ICRF`, `Fixed`, `Inertial`, `MeanOfEpoch`,
`MeanOfDate`, `TrueOfDate`, `TrueOfDateRotating` -- matching the same
inventory above. `core/frames.py::require_position_reference` validates
before a command is built.

## tf2

Published for visualisation/interoperability only, rooted at `icrf`.
Orbital positions (~7e6 m from origin) sit far outside the locally-flat,
bounded-world assumption most ROS tooling (RViz defaults, REP-105) was
tuned for. **The control loop never reads tf** -- it reads the typed
topics, in the frames their `frame_id` names. See
`interfaces/tf_broadcaster.py`'s module docstring.
