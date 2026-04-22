Create the initial ROS2 Python implementation for task_manager_node.

Requirements:
- Use rclpy
- Implement the workflow order exactly
- The manager should call abstract interfaces, not raw robot APIs directly
- Assume the following callable interfaces exist or will exist:
  - tool_manager.pick_tool(tool_name)
  - tool_manager.return_tool(tool_name)
  - object_manager.pick_and_place_plate(target_pose)
  - dough_task.place_dough_with_tongs()
  - dough_task.press_dough()
  - flip_task.flip_item_with_spatula()
  - sauce_task.dispense_sauce()
  - powder_task.sprinkle_powder()

Implementation constraints:
- Keep it readable
- Add clear logging
- Add basic success/failure handling
- Do not over-engineer
- If supporting files are needed, create them too

After coding:
1) summarize what files were created
2) summarize assumptions
3) list any TODO items clearly