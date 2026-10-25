"""Read-only inspection of the HPOP force model of the ACAV satellite.

    python scripts/inspect_hpop_force_model.py --output force_model.txt

Answers one question that orbit batching depends on
(``stk_bridge/core/orbit_batch.py``): can any force in the satellite's
HPOP model depend on its attitude? The scenario builder does not
configure the force model, so the answer lives only in the open STK
scenario and has to be read from it.

Strictly read-only. It attaches to an already running STK Desktop
(``STKDesktop.AttachToApplication()``, as the bridge does with
``attach_to_existing``) and never starts one, never calls ``ShutDown``
(which closes the scenario), never assigns a property, and sends no
Connect command. The complete list of Object Model operations is:

* ``app.Root``
* property reads, every one listed in :func:`inspect`
* method calls, all of them queries: ``ALLOWED_METHOD_CALLS``

``test_hpop_inspection.py`` checks this against a recording fake that
fails on any assignment or any other method call.

Every value is printed as STK returned it, in the session's current
units (also printed). A value that cannot be read through COM is printed
as ``UNREADABLE (<reason>)``; no default is ever substituted.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Callable, List, Optional, Sequence, Tuple

UNREADABLE = "UNREADABLE"

#: The only Object Model methods this module calls. All are queries.
ALLOWED_METHOD_CALLS = ("GetCurrentUnitAbbrv", "GetObjectFromPath", "Item")

#: Unit dimensions printed so the values below can be read correctly.
UNIT_DIMENSIONS = ("DateFormat", "Distance", "Mass", "Time", "Angle")

#: Model types whose force cannot depend on the vehicle's attitude: a
#: sphere presents the same cross-section in every orientation.
ATTITUDE_INDEPENDENT_MODELS = ("eDragModelSpherical", "eSRPModelSpherical")


def _format(value) -> str:
    """An enum as ``NAME (int)``, anything else as its repr."""
    name = getattr(value, "name", None)
    if isinstance(name, str):
        try:
            return f"{name} ({int(value)})"
        except (TypeError, ValueError):
            return name
    if isinstance(value, (bool, int, float, str)) or value is None:
        return repr(value)
    return f"{type(value).__name__}: {value}"


def _read(getter: Callable[[], object]) -> Tuple[str, object]:
    """(printed text, raw value or None) for one read."""
    try:
        value = getter()
    except Exception as error:                    # noqa: BLE001 - report it
        return f"{UNREADABLE} ({type(error).__name__}: {error})", None
    return _format(value), value


def inspect(root, satellite_path: str) -> List[Tuple[str, str]]:
    """Every reported quantity as (label, printed value)."""
    rows: List[Tuple[str, str]] = []
    raw = {}

    def add(label: str, getter: Callable[[], object]) -> None:
        text, value = _read(getter)
        rows.append((label, text))
        raw[label] = value

    add("scenario.InstanceName", lambda: root.CurrentScenario.InstanceName)
    add("scenario.StartTime", lambda: root.CurrentScenario.StartTime)
    add("scenario.StopTime", lambda: root.CurrentScenario.StopTime)
    add("scenario.Epoch", lambda: root.CurrentScenario.Epoch)
    for dimension in UNIT_DIMENSIONS:
        add(f"units.{dimension}", lambda d=dimension:
            root.UnitPreferences.GetCurrentUnitAbbrv(d))

    def sat():
        return root.GetObjectFromPath(satellite_path)

    def prop():
        return sat().Propagator

    def fm():
        return prop().ForceModel

    add("satellite.path", lambda: satellite_path)
    add("satellite.PropagatorType", lambda: sat().PropagatorType)
    add("satellite.AttitudeType", lambda: sat().AttitudeType)
    add("satellite.MassProperties.Mass", lambda: sat().MassProperties.Mass)
    add("propagator.type", lambda: type(prop()).__name__)
    add("propagator.Step", lambda: prop().Step)

    gravity = lambda: fm().CentralBodyGravity  # noqa: E731
    for name in ("File", "MaxDegree", "MaxOrder", "UseOceanTides",
                 "SolidTideType", "UseSecularVariations"):
        add(f"gravity.{name}", lambda n=name: getattr(gravity(), n))

    drag = lambda: fm().Drag  # noqa: E731
    add("drag.Use", lambda: drag().Use)
    add("drag.AtmosphericDensityModel", lambda: drag().AtmosphericDensityModel)
    add("drag.DragModelType", lambda: drag().DragModelType)
    add("drag.DragModel.type", lambda: type(drag().DragModel).__name__)
    add("drag.DragModel.Cd", lambda: drag().DragModel.Cd)
    add("drag.DragModel.AreaMassRatio", lambda: drag().DragModel.AreaMassRatio)

    srp = lambda: fm().SolarRadiationPressure  # noqa: E731
    add("srp.Use", lambda: srp().Use)
    add("srp.ShadowModel", lambda: srp().ShadowModel)
    add("srp.SRPModel.ModelType", lambda: srp().SRPModel.ModelType)
    add("srp.SRPModel.Model.type", lambda: type(srp().SRPModel.Model).__name__)
    add("srp.SRPModel.Model.Cr", lambda: srp().SRPModel.Model.Cr)
    add("srp.SRPModel.Model.AreaMassRatio",
        lambda: srp().SRPModel.Model.AreaMassRatio)

    more = lambda: fm().MoreOptions  # noqa: E731
    add("more.Drag.UseApproxAlt", lambda: more().Drag.UseApproxAlt)
    add("more.Drag.UseApparentSunPosition",
        lambda: more().Drag.UseApparentSunPosition)
    add("more.SolarRadiationPressure.MethodToComputeSunPosition",
        lambda: more().SolarRadiationPressure.MethodToComputeSunPosition)
    for name in ("Static", "SolidTides", "OceanTides", "RadiationPressure",
                 "PluginPropagator"):
        add(f"more.{name}", lambda n=name: getattr(more(), n))
    add("force_model.EclipsingBodies", lambda: fm().EclipsingBodies)

    add("third_body.Count", lambda: fm().ThirdBodyGravity.Count)
    count = raw.get("third_body.Count")
    if isinstance(count, int) and not isinstance(count, bool):
        for index in range(count):
            for name in ("CentralBody", "Source", "GravityValue"):
                add(f"third_body[{index}].{name}",
                    lambda i=index, n=name:
                    getattr(fm().ThirdBodyGravity.Item(i), n))

    rows.append(("attitude_dependent_force_assessment",
                 _assess(raw)))
    return rows


def _assess(raw) -> str:
    """NO / POSSIBLE / UNDETERMINED, from the drag and SRP settings only.

    A rule over the values printed above, not a further STK query: a
    force can depend on attitude only if it is in use and its model is
    not spherical. Plug-in and GPS-specific SRP models are reported as
    POSSIBLE, because what they assume about attitude is not visible
    through these properties.
    """
    findings = []
    for force, use_key, type_key in (
            ("drag", "drag.Use", "drag.DragModelType"),
            ("srp", "srp.Use", "srp.SRPModel.ModelType")):
        use = raw.get(use_key)
        if not isinstance(use, bool):
            return f"UNDETERMINED ({use_key} unreadable)"
        if not use:
            findings.append(f"{force} off")
            continue
        model = getattr(raw.get(type_key), "name", None)
        if not isinstance(model, str):
            return f"UNDETERMINED ({type_key} unreadable)"
        if model not in ATTITUDE_INDEPENDENT_MODELS:
            return f"POSSIBLE ({force} uses {model})"
        findings.append(f"{force} {model}")
    return "NO (" + ", ".join(findings) + ")"


def _attach():
    """Attach to a running STK Desktop. Never starts one."""
    from agi.stk12.stkdesktop import STKDesktop

    return STKDesktop.AttachToApplication()


def main(argv: Optional[Sequence[str]] = None,
         attach: Optional[Callable[[], object]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only HPOP force-model inspection.")
    parser.add_argument("--output", required=True,
                        help="text file to write; must not already exist")
    parser.add_argument("--satellite", default="*/Satellite/SSS-1")
    args = parser.parse_args(argv)

    if os.path.exists(args.output):
        print(f"{args.output} already exists; choose a new file",
              file=sys.stderr)
        return 2
    try:
        app = (attach or _attach)()
        root = app.Root
    except Exception as error:                    # noqa: BLE001
        print(f"could not attach to a running STK Desktop: {error}",
              file=sys.stderr)
        return 2

    lines = [f"{label} = {value}" for label, value
             in inspect(root, args.satellite)]
    text = "\n".join(lines) + "\n"
    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(text)
    sys.stdout.write(text)
    return 0
