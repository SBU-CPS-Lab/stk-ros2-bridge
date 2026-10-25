"""stk_bridge -- bidirectional ROS 2 bridge for AGI/Ansys STK 12.6.

Subpackages
-----------
backends    STK access implementations (com, engine, connect, mock) behind
            the StkBackend abstract interface. See docs/bridge/extending_backends.md.
core        Backend-agnostic domain logic: unit/frame conversion, sensing
            pipeline, actuation, time synchronization, scenario management.
interfaces  ROS 2-facing glue: publishers, subscribers, services, actions, tf.
nodes       Executable entry points (bridge_node, clock_node, diagnostics_node).

Nothing outside ``backends/`` may import ``agi.stk12``, ``win32com`` or
any other STK/COM-specific library; see docs/bridge/architecture.md.
"""

__version__ = "0.1.0"
