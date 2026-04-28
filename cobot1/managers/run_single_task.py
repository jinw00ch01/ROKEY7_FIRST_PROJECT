"""
run_single_task.py
개별 태스크를 새 아키텍처(ToolManager + tasks + IOManager)로 실행.

사용법:
  ros2 run cobot1 run_task --ros-args -p task:=press
  ros2 run cobot1 run_task --ros-args -p task:=dough
  ros2 run cobot1 run_task --ros-args -p task:=plate
  ros2 run cobot1 run_task --ros-args -p task:=spatula
  ros2 run cobot1 run_task --ros-args -p task:=source
  ros2 run cobot1 run_task --ros-args -p task:=powder
"""
import rclpy
import DR_init
from rclpy.node import Node

from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import ROBOT_CONFIG
from cobot1.managers.tool_manager import ToolManager
from cobot1.managers.object_manager import ObjectManager
from cobot1.tasks.dough_task import DoughTask
from cobot1.tasks.press_task import PressTask
from cobot1.tasks.flip_task import FlipTask
from cobot1.tasks.source_task import SourceTask
from cobot1.tasks.powder_task import PowderTask
from cobot1.tasks.plate_setting_task import PlateSettingTask

TASK_MAP = {
    'dough':   ('tongs',        '집게로 반죽 배치'),
    'press':   ('presser',      '프레스로 반죽 누르기'),
    'plate':   (None,           '접시 배치'),
    'spatula': ('spatula',      '뒤집개로 뒤집기'),
    'source':   ('source_bottle', '소스 뿌리기'),
    'powder':  ('powder_bottle','가루 뿌리기'),
}


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_CONFIG['robot_id']
    DR_init.__dsr__model = ROBOT_CONFIG['robot_model']

    node = rclpy.create_node('run_single_task',
                             namespace=ROBOT_CONFIG['robot_id'])
    DR_init.__dsr__node = node

    node.declare_parameter('task', '')
    task_name = node.get_parameter('task').get_parameter_value().string_value

    if task_name not in TASK_MAP:
        node.get_logger().error(
            f'Unknown task: "{task_name}". '
            f'Available: {", ".join(TASK_MAP.keys())}')
        node.destroy_node()
        rclpy.shutdown()
        return

    tool_name, desc = TASK_MAP[task_name]
    node.get_logger().info(f'Running single task: {desc}')

    # 로봇 초기화
    io = IOManager(node)
    io.set_tool_tcp()
    io.set_robot_mode_autonomous()

    tool_mgr = ToolManager(node, io=io)

    try:
        if task_name == 'dough':
            tool_mgr.pick_tool(tool_name)
            DoughTask(node, io=io).place_dough_with_tongs()
            tool_mgr.return_tool(tool_name)

        elif task_name == 'press':
            tool_mgr.pick_tool(tool_name)
            PressTask(node, io=io).press_dough()
            tool_mgr.return_tool(tool_name)

        elif task_name == 'plate':
            PlateSettingTask(node, io=io).pick_and_place_plate()

        elif task_name == 'spatula':
            tool_mgr.pick_tool(tool_name)
            FlipTask(node, io=io).flip_item_with_spatula()
            tool_mgr.return_tool(tool_name)

        elif task_name == 'source':
            tool_mgr.pick_tool(tool_name)
            SourceTask(node, io=io).dispense_source()
            tool_mgr.return_tool(tool_name)

        elif task_name == 'powder':
            tool_mgr.pick_tool(tool_name)
            PowderTask(node, io=io).sprinkle_powder()
            tool_mgr.return_tool(tool_name)

        node.get_logger().info(f'Task "{task_name}" completed!')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
