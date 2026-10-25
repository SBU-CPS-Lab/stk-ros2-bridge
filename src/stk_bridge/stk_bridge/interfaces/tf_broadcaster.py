"""tf2 broadcasting for orbital and vehicle frames.

Publishes a tree rooted at ``icrf`` so standard ROS tooling (RViz,
``tf2_echo``, ``view_frames``) works against an orbital scenario.

A caveat worth stating plainly
------------------------------
tf2 stores translations as float64 metres, which is fine, but tf2's
buffer and many downstream consumers assume a *locally-flat, bounded*
world of the sort REP-105 describes. An orbital position is ~7e6 m from
the origin, which is representable but sits far outside the range most
robotics tooling was tuned for -- RViz's default view scaling and grid
sizes in particular are useless without adjustment.

So tf here is for interoperability and visualisation, not the numerical
path. The authoritative state is on the typed topics, in the frames
their ``frame_id`` names. Nothing in the control loop reads tf.

Static transforms
-----------------
The inertial-to-inertial relationships (``icrf`` -> ``mean_of_epoch``,
etc.) are *not* published as identity. They are small but non-zero
rotations, and publishing identity would assert they are the same
frame. Where the bridge cannot compute the true rotation it omits the
edge, leaving the tree disconnected there, which ``tf2_echo`` reports.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional

from stk_bridge.backends.base import AttitudeSample, StateSample, StkTime
from stk_bridge.core import conversions as conv
from stk_bridge.core.frames import FrameRegistry

LOGGER = logging.getLogger(__name__)


class TfBroadcaster:
    """Publishes vehicle pose into tf2."""

    def __init__(self, node, frames: FrameRegistry,
                 publish_body: bool = True,
                 publish_position: bool = True):
        from tf2_ros import TransformBroadcaster

        self._node = node
        self._frames = frames
        self._broadcaster = TransformBroadcaster(node)
        self._publish_body = publish_body
        self._publish_position = publish_position
        self._sent = 0

    @property
    def sent_count(self) -> int:
        return self._sent

    def _transform(self, when: StkTime, parent: str, child: str):
        from geometry_msgs.msg import TransformStamped

        transform = TransformStamped()
        transform.header.stamp = conv.stk_time_to_ros(when)
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def broadcast(self, object_path: str,
                  orbit: Optional[StateSample] = None,
                  attitude: Optional[AttitudeSample] = None) -> int:
        """Publish this vehicle's transforms. Returns the number sent.

        The vehicle origin frame carries position only; the body frame
        carries orientation relative to it. Splitting them means a
        missing attitude sample does not suppress the position
        transform, and vice versa -- each is published only when its
        source data exists.
        """
        transforms = []

        origin_frame = self._frames.vehicle_frame_id(object_path, "icrf")

        if orbit is not None and self._publish_position:
            transform = self._transform(orbit.time, orbit.frame, origin_frame)
            transform.transform.translation.x = orbit.position_m[0]
            transform.transform.translation.y = orbit.position_m[1]
            transform.transform.translation.z = orbit.position_m[2]
            transforms.append(transform)

        if attitude is not None and self._publish_body:
            body_frame = self._frames.vehicle_frame_id(object_path, "body")
            # Parent is the frame the attitude is actually expressed in
            # (fixed for ECF Attitude Quaternions, icrf for the inertial
            # provider), not the vehicle origin -- attaching it to the
            # wrong parent is the tf equivalent of the ECI/ECF pairing
            # error that validation exists to catch.
            transform = self._transform(attitude.time, attitude.frame,
                                        body_frame)
            qx, qy, qz, qw = attitude.quaternion_xyzw
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        if transforms:
            self._broadcaster.sendTransform(transforms)
            self._sent += len(transforms)
        return len(transforms)

    def broadcast_samples(self, object_path: str,
                          samples: Dict[str, object]) -> int:
        return self.broadcast(
            object_path,
            orbit=samples.get("orbit"),
            attitude=samples.get("attitude"),
        )
