Create a robot control wrapper module for this project.

Goals:
- Centralize robot motion and I/O calls
- Expose safe, simple functions for the rest of the project

Include wrapper functions such as:
- move_joint_safe(...)
- move_line_safe(...)
- gripper_open(...)
- gripper_close(...)
- wait_input_ok(...)
- set_tool_tcp(...)

Constraints:
- Keep ROS2 service interaction isolated in this module
- Add docstrings
- Add clear placeholders where concrete Doosan service details must be connected
- Do not fake unavailable response fields
- Prefer explicit parameter names

Also generate a short example showing how task nodes should use this wrapper.