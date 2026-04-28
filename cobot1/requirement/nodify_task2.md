Based on this workspace, design a ROS2 node architecture for the workflow.

Requirements:
- Separate managers from task nodes
- Include:
  - task_manager_node
  - tool_manager_node
  - object_manager_node
  - io_manager helper or module
  - pose_manager helper or module
  - task nodes:
    - place_dough_with_tongs_node
    - press_dough_node
    - pick_and_place_plate_node
    - flip_item_with_spatula_node
    - dispense_source_node
    - sprinkle_powder_node
- Tool-specific handling must be separated internally for:
  - tongs
  - presser
  - spatula
  - source container
  - powder container

Output format:
1) tree-style file structure
2) responsibility of each file/module
3) data flow between nodes/modules
4) state variables that must be shared or tracked
Do not write implementation code yet.