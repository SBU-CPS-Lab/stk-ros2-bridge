"""A MockBackend that reproduces STK 12.6's *measured* misbehaviour.

Everything here was observed on a Windows STK 12.6 install, and every
one of them produced a scenario that built cleanly and measured the wrong
thing:

* ``SetPosition ... Geodetic`` consumes **degrees**, although
  ``configure_units()`` set the Angle preference to radians and STK
  accepted it. Sending radians put all seven targets over the wrong
  part of the planet, and the builder's own arithmetic check passed
  because it never asked STK where they had landed.
* ``SetState ... Classical`` also consumes degrees, and accepts only
  the ``<start> <stop> <step>`` argument order. Sending
  ``radians(97.4)`` produced a satellite at 1.700 deg inclination --
  a perfectly plausible orbit, along whose ground track the targets
  were then placed.
* ``Define ... SimpleConic`` is rejected in both angle units, leaving
  the sensor at STK's wide default half-angle.
* ``SetAzElMask`` is rejected in every form, so neither terrain nor
  constant masking is applied.
* A freshly created satellite starts on STK's default orbit, not the
  one the study wants -- so a SetState that silently fails is
  indistinguishable from one that worked unless somebody measures.

A mock that is kinder than production is worse than no mock. Once a
failure has cost a live round trip, it belongs here so it can never
cost a second one.
"""

import math
import shlex
from typing import List, Optional

from stk_bridge.backends.base import (
    AccessIntervalData,
    ConnectResponse,
    StkTime,
)
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.exceptions import CommandError

#: STK's default satellite orbit -- deliberately not the study's.
STK_DEFAULT_ORBIT = {
    "semi_major_axis_m": 7_163_000.0,
    "eccentricity": 0.0,
    "inclination_rad": math.radians(28.5),
    "raan_rad": 0.0,
    "arg_perigee_rad": 0.0,
    "true_anomaly_rad": 0.0,
}

#: What a SimpleConic sensor is until something narrows it.
STK_DEFAULT_CONE_HALF_ANGLE_RAD = math.radians(45.0)

#: Effective horizon a loaded terrain mask imposes, degrees. Stands in
#: for real terrain so a terrain-vs-constant comparison has something to
#: compare.
TERRAIN_HORIZON_DEG = 15.0


class Stk126Double(MockBackend):
    """MockBackend plus the quirks measured on the live 12.6 install."""

    def __init__(self, *args, reject_define: bool = True,
                 reject_azelmask: bool = True,
                 geodetic_unit: str = "degrees",
                 setstate_unit: str = "degrees",
                 lla_unit: str = "radians",
                 setstate_order: str = "start_stop_step", **kwargs):
        super().__init__(*args, **kwargs)
        self.reject_define = reject_define
        self.reject_azelmask = reject_azelmask
        self.geodetic_unit = geodetic_unit
        self.setstate_unit = setstate_unit
        self.lla_unit = lla_unit
        self.setstate_order = setstate_order
        #: Effective horizon from the applied mask, degrees.
        #: ``TERRAIN_HORIZON_DEG`` stands in for terrain having been
        #: loaded for the region.
        self.mask_horizon_deg: float = 0.0
        self.mask_mode: Optional[str] = None
        #: Minimum-elevation access constraint, degrees. Separate from
        #: the mask, because in STK it is a separate mechanism -- and
        #: because a terrain horizon above it hides its effect, which is
        #: a confound the verifier has to control for.
        self.elevation_min_deg: float = 0.0
        #: Every Connect command issued, so a test can assert on the
        #: unit a command was sent in rather than only on its effect.
        self.commands: List[str] = []

    def _lla_columns(self, obj):
        """Report LLA in whichever unit this install is pretending to use.

        The read direction is a separate question from the write
        direction -- a live run showed Connect consuming degrees while
        the Angle *preference* was radians -- so it gets its own knob
        rather than being derived from geodetic_unit.
        """
        inner = super()._lla_columns(obj)
        if self.lla_unit == "radians":
            return inner

        def build(when: float):
            columns = inner(when)
            return {key: (math.degrees(value) if key in ("Lat", "Lon")
                          else value)
                    for key, value in columns.items()}
        return build

    def compute_access(self, from_path: str, to_path: str, start, stop):
        """Ground-station contact that actually responds to the mask.

        Not a geometry model -- just enough for a mask comparison to
        mean something. Without it a test of "does the mask change
        access" is answered by a constant, which is precisely the
        non-experiment an earlier terrain verdict rested on.
        """
        facility = any(self._objects.get(p) is not None
                       and self._objects[p].object_type == "Facility"
                       for p in (from_path, to_path))
        if not facility:
            return super().compute_access(from_path, to_path, start, stop)

        # Whichever occludes more wins, exactly as a real horizon and a
        # minimum-elevation constraint compose.
        elevation = max(self.mask_horizon_deg, self.elevation_min_deg)
        seconds = max(0.0, 600.0 - 20.0 * elevation)
        if seconds <= 0.0:
            return []
        begin = start.epoch_seconds
        end = min(begin + seconds, stop.epoch_seconds)
        return [AccessIntervalData(StkTime(begin), StkTime(end))]

    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        path = super().create_object(object_type, name, parent_path)
        obj = self._objects[path]
        if object_type == "Satellite":
            # No ephemeris of the *requested* orbit until SetState works.
            obj.elements = dict(STK_DEFAULT_ORBIT)
        if object_type == "Sensor":
            obj.cone_half_angle_rad = STK_DEFAULT_CONE_HALF_ANGLE_RAD
        return path

    def set_az_el_mask(self, object_path: str, mode: str,
                       value: float = 0.0) -> str:
        """STK's three mask types, and nothing else.

        A live install reports exactly eAzElMaskNone, eMaskFile and
        eTerrainData -- there is no constant-elevation mask. Rejecting
        anything else here is how the double stops the verifier from
        going back to using one as a control.
        """
        if mode not in ("terrain", "none", "file"):
            raise CommandError("SetAzElMask", object_path,
                               f"unknown mask mode {mode!r}; "
                               f"AgEAzElMaskType members: "
                               f"['eAzElMaskNone', 'eMaskFile', "
                               f"'eTerrainData']")
        if self.reject_azelmask:
            raise CommandError(
                "SetAzElMask", object_path,
                "Object Model SetAzElMask [eTerrainData]: "
                "no such enum member; "
                "Connect SetAzElMask TerrainData: NACK Command has failed.")
        self.mask_horizon_deg = (TERRAIN_HORIZON_DEG if mode == "terrain"
                                 else 0.0)
        self.mask_mode = mode
        return f"double (stored {mode})"

    def get_az_el_mask(self, object_path: str) -> Optional[str]:
        return self.mask_mode

    def set_sensor_cone(self, object_path: str, half_angle_rad: float,
                        angular_resolution_rad: float = 0.017453) -> str:
        if self.reject_define:
            raise CommandError(
                "Define SimpleConic", object_path,
                "Connect Define SimpleConic (degrees): NACK; "
                "Connect Define SimpleConic (radians): NACK")
        return super().set_sensor_cone(object_path, half_angle_rad,
                                       angular_resolution_rad)

    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
        self.commands.append(command)
        tokens = shlex.split(command)
        verb = tokens[0].lower() if tokens else ""

        if verb == "define" and self.reject_define:
            raise CommandError("Define", command, "Command has failed.")
        if verb == "setazelmask":
            # Connect is rejected outright on the live install; the
            # typed path in set_az_el_mask is the one that works.
            raise CommandError("SetAzElMask", command,
                               "Command has failed.")
        if verb == "setconstraint" and len(tokens) >= 5 \
                and tokens[2].lower() == "elevationangle":
            self.elevation_min_deg = float(tokens[4])
            return ConnectResponse(command=command, ack=True, messages=[],
                                   raw="ACK")
        if verb == "setstate":
            return self._set_state(tokens, command)
        if verb == "setposition" and len(tokens) >= 6 \
                and tokens[2].lower() == "geodetic":
            return self._set_position_geodetic(tokens, command)
        return super().execute_connect(command, timeout_s)

    # -- quirks ------------------------------------------------------------

    def _angle_to_rad(self, value: float, unit: str) -> float:
        return math.radians(value) if unit == "degrees" else value

    def _set_position_geodetic(self, tokens, command) -> ConnectResponse:
        obj = self._objects.get(tokens[1])
        if obj is None:
            raise CommandError("SetPosition", tokens[1], "no such object")
        latitude, longitude, altitude = (float(t) for t in tokens[3:6])
        obj.geodetic = (self._angle_to_rad(latitude, self.geodetic_unit),
                        self._angle_to_rad(longitude, self.geodetic_unit),
                        altitude)
        return ConnectResponse(command=command, ack=True, messages=[],
                               raw="ACK")

    def _set_state(self, tokens, command) -> ConnectResponse:
        # SetState <path> Classical <prop> <a> <b> <c> <frame> <epoch> x6
        if len(tokens) < 15 or tokens[2].lower() != "classical":
            raise CommandError("SetState", command, "Command has failed.")
        obj = self._objects.get(tokens[1])
        if obj is None:
            raise CommandError("SetState", tokens[1], "no such object")

        first_is_step = _is_number(tokens[4])
        order = "step_start_stop" if first_is_step else "start_stop_step"
        if order != self.setstate_order:
            raise CommandError("SetState", command, "Command has failed.")

        values = [float(t) for t in tokens[9:15]]
        obj.elements = {
            "semi_major_axis_m": values[0],
            "eccentricity": values[1],
            "inclination_rad": self._angle_to_rad(values[2],
                                                  self.setstate_unit),
            "arg_perigee_rad": self._angle_to_rad(values[3],
                                                  self.setstate_unit),
            "raan_rad": self._angle_to_rad(values[4], self.setstate_unit),
            "true_anomaly_rad": self._angle_to_rad(values[5],
                                                   self.setstate_unit),
        }
        return ConnectResponse(command=command, ack=True, messages=[],
                               raw="ACK")


def _is_number(token: str) -> bool:
    try:
        float(token)
    except ValueError:
        return False
    return True
