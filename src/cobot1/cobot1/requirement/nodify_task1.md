You are helping me build a ROS2-based task system for a Doosan robot.

Project goal:
- Build a task-oriented node architecture for a food-process workflow.
- The workflow order is:
  1) pick tongs
  2) place dough with tongs
  3) return tongs
  4) pick presser
  5) press dough
  6) return presser
  7) pick and place plate to target position
  8) pick spatula
  9) flip item with spatula
  10) return spatula
  11) pick sauce container
  12) dispense sauce
  13) return sauce container
  14) pick powder container
  15) sprinkle powder
  16) return powder container

Constraints:
- ROS2 Python package
- Use clear node separation
- Tool pick/return must support different gripping strategies per tool
- Plate is not a tool; it is an object pick-and-place task
- The robot uses a gripper through I/O
- Prioritize maintainability and clear state transitions
- Use movej for large safe moves and movel for approach/insert/retreat patterns unless there is a strong reason otherwise
- Do not invent unavailable robot APIs; keep robot calls abstract behind wrapper functions if needed

What I want from you:
1) Inspect the current workspace structure first
2) Propose package/module/file structure
3) Explain the architecture briefly
4) Then wait for my next instruction before generating code