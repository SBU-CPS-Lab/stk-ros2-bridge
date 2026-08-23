"""Reference-frame naming, aliasing, and the tf2 tree.

STK and ROS name frames differently and disagree about what a frame tree
is for. STK frames are astrodynamic and global (ICRF, J2000, Fixed,
LVLH, VVLH, Body); ROS's REP-105 frames are robot-local (``map``,
``odom``, ``base_link``) and assume a locally-flat world. Forcing orbital
frames into REP-105 names would be actively misleading -- an orbit is not
an ``odom`` frame -- so this module does not do that by default.

Instead:

* Every published message carries an explicit ``frame_id`` naming the
  actual STK frame, lower-cased (``icrf``, ``j2000``, ``fixed``,
  ``lvlh``, ``body``).
* A tf2 tree is broadcast with ``icrf`` at the root, so standard ROS
  tooling (RViz, ``tf2_echo``) works.
* ``config/frames.yaml`` may declare REP-105 aliases for consumers that
  insist on them, and those are applied as *additional* names, never as
  replacements.

REP-103 compliance is about units and handedness, and both hold here:
metres, seconds, radians, right-handed, quaternion ``(x, y, z, w)``. The
axis-orientation convention in REP-103 (x-forward, y-left, z-up) is a
statement about *body* frames of ground robots and simply does not apply
to an inertial astrodynamic frame; the body frame's relationship to the
spacecraft structure is a mission definition, declared in
``config/frames.yaml`` rather than assumed.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

from stk_bridge.exceptions import FrameResolutionError

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class FrameDefinition:
    """One reference frame known to the bridge."""

    #: Canonical ROS ``frame_id`` (lower-case, no spaces).
    ros_id: str
    #: The exact string STK expects in Connect commands and data-provider
    #: group members. Case matters to STK here.
    stk_name: str
    #: Parent in the tf tree, or None for the root.
    parent: Optional[str] = None
    #: True if this frame is inertial (non-rotating). Used to decide
    #: whether angular velocity needs a transport term, and to reject
    #: obviously-wrong pairings like commanding inertial attitude into a
    #: central-body-fixed channel.
    inertial: bool = True
    #: True if this frame is attached to a vehicle and therefore only
    #: meaningful with an object path qualifier.
    vehicle_relative: bool = False
    description: str = ""


#: Frames the bridge understands out of the box. ``config/frames.yaml``
#: can add to this and declare aliases, but these are always present so
#: that a bridge with no config file still works.
BUILTIN_FRAMES: Dict[str, FrameDefinition] = {
    "icrf": FrameDefinition(
        ros_id="icrf", stk_name="ICRF", parent=None, inertial=True,
        description="International Celestial Reference Frame. Tree root.",
    ),
    "j2000": FrameDefinition(
        ros_id="j2000", stk_name="J2000", parent="icrf", inertial=True,
        description="Mean equator and equinox of J2000. Differs from ICRF "
                    "by the frame bias (~20 mas) -- not zero, but far "
                    "below anything this bridge resolves.",
    ),
    "fixed": FrameDefinition(
        ros_id="fixed", stk_name="Fixed", parent="icrf", inertial=False,
        description="Central-body-fixed (ECEF for Earth). The reference "
                    "axes for AddAttitude CBFQuat.",
    ),
    "inertial": FrameDefinition(
        ros_id="inertial", stk_name="Inertial", parent="icrf", inertial=True,
        description="STK's generic central-body inertial axes.",
    ),
    "lvlh": FrameDefinition(
        ros_id="lvlh", stk_name="LVLH", parent="icrf", inertial=False,
        vehicle_relative=True,
        description="Local Vertical Local Horizontal, vehicle-relative.",
    ),
    "vvlh": FrameDefinition(
        ros_id="vvlh", stk_name="VVLH", parent="icrf", inertial=False,
        vehicle_relative=True,
        description="Vehicle Velocity Local Horizontal. STK's nearest "
                    "analogue to aircraft yaw/pitch/roll axes.",
    ),
    "body": FrameDefinition(
        ros_id="body", stk_name="Body", parent="icrf", inertial=False,
        vehicle_relative=True,
        description="Spacecraft body axes. The frame ADCS actually "
                    "controls; its relationship to structure is a mission "
                    "definition, not a convention this bridge can assume.",
    ),
}

#: Frames valid as the reference axes for real-time attitude injection.
#: ``AddAttitude CBFQuat`` takes central-body-fixed axes and
#: ``AddAttitude Quat`` takes inertial axes -- there is no third option,
#: so commanding attitude relative to LVLH requires the caller to rotate
#: into one of these first.
ATTITUDE_REFERENCE_FRAMES: Set[str] = {"fixed", "icrf", "j2000", "inertial"}

#: Frames valid as the reference for ``SetPosition``.
POSITION_REFERENCE_FRAMES: Set[str] = {"icrf", "j2000", "fixed", "inertial"}


class FrameRegistry:
    """Resolves frame names and owns the tf tree definition."""

    def __init__(self, definitions: Optional[Dict[str, FrameDefinition]] = None,
                 aliases: Optional[Dict[str, str]] = None,
                 tf_prefix: str = ""):
        self._frames: Dict[str, FrameDefinition] = dict(
            definitions if definitions is not None else BUILTIN_FRAMES
        )
        self._aliases: Dict[str, str] = {}
        self._tf_prefix = tf_prefix.strip("/")

        # Case-insensitive lookup by STK name, so callers can pass either
        # naming convention without knowing which they hold.
        for frame in self._frames.values():
            self._aliases[frame.stk_name.lower()] = frame.ros_id

        for alias, target in (aliases or {}).items():
            self.add_alias(alias, target)

    # -- construction ------------------------------------------------------

    @classmethod
    def from_config(cls, config: Optional[Dict] = None) -> "FrameRegistry":
        """Build from a parsed ``frames.yaml`` mapping.

        Expected shape::

            frames:
              tf_prefix: ""
              aliases:
                map: icrf
                base_link: body
              additional:
                topocentric:
                  stk_name: Topocentric
                  parent: fixed
                  inertial: false
                  vehicle_relative: true
        """
        config = (config or {}).get("frames", {}) or {}
        definitions = dict(BUILTIN_FRAMES)

        for ros_id, spec in (config.get("additional") or {}).items():
            key = str(ros_id).strip().lower()
            definitions[key] = FrameDefinition(
                ros_id=key,
                stk_name=str(spec.get("stk_name", ros_id)),
                parent=(str(spec["parent"]).lower()
                        if spec.get("parent") else None),
                inertial=bool(spec.get("inertial", True)),
                vehicle_relative=bool(spec.get("vehicle_relative", False)),
                description=str(spec.get("description", "")),
            )

        return cls(
            definitions=definitions,
            aliases=config.get("aliases") or {},
            tf_prefix=str(config.get("tf_prefix", "")),
        )

    def add_alias(self, alias: str, target: str) -> None:
        """Register an additional name for an existing frame.

        Aliases are additive. An alias that would shadow a real frame's
        canonical id is rejected, because silently redirecting ``icrf`` to
        something else would corrupt every message that names it.
        """
        alias_key = str(alias).strip().lower()
        target_key = str(target).strip().lower()
        if target_key not in self._frames:
            raise FrameResolutionError(target)
        if alias_key in self._frames:
            raise ValueError(
                f"alias {alias!r} would shadow the canonical frame of the "
                f"same name; choose a different alias"
            )
        self._aliases[alias_key] = target_key

    # -- resolution --------------------------------------------------------

    def resolve(self, frame_id: str) -> FrameDefinition:
        """Look up a frame by ROS id, STK name, or alias."""
        key = str(frame_id or "").strip().lower().lstrip("/")
        if self._tf_prefix and key.startswith(self._tf_prefix + "/"):
            key = key[len(self._tf_prefix) + 1:]
        if key in self._frames:
            return self._frames[key]
        if key in self._aliases:
            return self._frames[self._aliases[key]]
        raise FrameResolutionError(frame_id)

    def stk_name(self, frame_id: str) -> str:
        """The exact string to hand STK for this frame."""
        return self.resolve(frame_id).stk_name

    def ros_id(self, frame_id: str) -> str:
        """The canonical ROS ``frame_id``, with tf prefix applied."""
        base = self.resolve(frame_id).ros_id
        return f"{self._tf_prefix}/{base}" if self._tf_prefix else base

    def known_frames(self) -> List[str]:
        return sorted(self._frames)

    # -- validation --------------------------------------------------------

    def require_attitude_reference(self, frame_id: str) -> FrameDefinition:
        """Validate a frame as an attitude-command reference.

        Rejecting an unsupported frame here, before a command is built,
        turns a would-be silent misinterpretation into a clear error.
        Commanding a quaternion expressed in LVLH through the CBFQuat
        channel does not fail in STK -- it is accepted and interpreted as
        central-body-fixed, producing an attitude that is wrong by the
        LVLH-to-fixed rotation and looks entirely plausible on screen.
        """
        frame = self.resolve(frame_id)
        if frame.ros_id not in ATTITUDE_REFERENCE_FRAMES:
            raise FrameResolutionError(
                f"{frame_id!r} is not a valid attitude reference frame. "
                f"STK's real-time attitude channel accepts only "
                f"central-body-fixed (AddAttitude CBFQuat) or inertial "
                f"(AddAttitude Quat) axes; valid values are "
                f"{sorted(ATTITUDE_REFERENCE_FRAMES)}. To command "
                f"attitude relative to {frame.ros_id}, rotate into one of "
                f"these first."
            )
        return frame

    def require_position_reference(self, frame_id: str) -> FrameDefinition:
        frame = self.resolve(frame_id)
        if frame.ros_id not in POSITION_REFERENCE_FRAMES:
            raise FrameResolutionError(
                f"{frame_id!r} is not a valid SetPosition reference frame; "
                f"valid values are {sorted(POSITION_REFERENCE_FRAMES)}"
            )
        return frame

    def is_inertial_attitude_reference(self, frame_id: str) -> bool:
        """True if attitude in this frame maps to ``AddAttitude Quat``.

        The direct answer to the ``inertial`` flag on
        ``stk_msgs/AttitudeCommand``: inertial frames take ``Quat``,
        central-body-fixed takes ``CBFQuat``.
        """
        return self.require_attitude_reference(frame_id).inertial

    # -- tf tree -----------------------------------------------------------

    def tf_parent(self, frame_id: str) -> Optional[str]:
        frame = self.resolve(frame_id)
        if frame.parent is None:
            return None
        return (f"{self._tf_prefix}/{frame.parent}" if self._tf_prefix
                else frame.parent)

    def root_frame(self) -> str:
        for frame in self._frames.values():
            if frame.parent is None:
                return self.ros_id(frame.ros_id)
        return self.ros_id("icrf")

    def vehicle_frame_id(self, object_path: str, frame_id: str = "body") -> str:
        """Per-vehicle tf frame name, e.g. ``sss_1/body``.

        Vehicle-relative frames are meaningless without knowing which
        vehicle, and tf is a single global namespace, so the object name
        is folded into the frame id. Non-alphanumerics become
        underscores because tf frame ids with slashes have historically
        been a source of trouble in tf2.
        """
        from stk_bridge.core.conversions import object_name

        name = object_name(object_path)
        safe = "".join(c if c.isalnum() else "_" for c in name).strip("_")
        base = self.resolve(frame_id).ros_id
        prefix = f"{self._tf_prefix}/" if self._tf_prefix else ""
        return f"{prefix}{safe}/{base}"

    def __repr__(self) -> str:
        return (f"<FrameRegistry {len(self._frames)} frames, "
                f"{len(self._aliases)} aliases, root={self.root_frame()!r}>")
