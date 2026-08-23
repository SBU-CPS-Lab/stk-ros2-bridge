"""ROS 2-facing glue.

Everything here imports ``rclpy`` and ``stk_msgs``; nothing in ``core/``
does. That separation is what keeps the core unit-testable without a ROS
installation, so the imports are kept inside functions and constructors
rather than at module scope.
"""
