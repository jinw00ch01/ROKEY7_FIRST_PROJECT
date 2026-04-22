"""
task_manager_node.py
워크플로우 전체를 순서대로 제어하는 ROS2 노드.

- 추상 인터페이스(tool_manager, object_manager, task modules)만 호출
- io_manager를 통해 로봇 제어, 직접 로봇 API 호출 없음
- ROS2 서비스로 외부(robot_backend 등)에서 워크플로우 트리거 가능
"""
import rclpy
import DR_init
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import IntEnum

from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import ROBOT_CONFIG
from cobot1.managers.tool_manager import ToolManager
from cobot1.managers.object_manager import ObjectManager
from cobot1.tasks.dough_task import DoughTask
from cobot1.tasks.press_task import PressTask
from cobot1.tasks.flip_task import FlipTask
from cobot1.tasks.sauce_task import SauceTask
from cobot1.tasks.powder_task import PowderTask


class WorkflowState(IntEnum):
    IDLE = 0
    RUNNING = 1
    PAUSED = 2
    ERROR = 3
    COMPLETED = 4


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node',
                         namespace=ROBOT_CONFIG['robot_id'])

        # 파라미터
        self.declare_parameter('sauce', 'none')
        self.declare_parameter('powder', 'none')

        # 상태 퍼블리셔
        self.status_pub = self.create_publisher(String, 'workflow_status', 10)

        # IO 매니저 (로봇 모션 + 그리퍼 통합)
        # 주의: set_tool_tcp/set_robot_mode_autonomous는 run_workflow()에서 호출
        #       (DSR_ROBOT2는 DR_init.__dsr__node 설정 후에만 import 가능)
        self.io = IOManager(self)

        # 매니저 & 태스크 모듈 (io를 공유)
        self.tool_manager = ToolManager(self, io=self.io)
        self.object_manager = ObjectManager(self, io=self.io)
        self.dough_task = DoughTask(self, io=self.io)
        self.press_task = PressTask(self, io=self.io)
        self.flip_task = FlipTask(self, io=self.io)
        self.sauce_task = SauceTask(self, io=self.io)
        self.powder_task = PowderTask(self, io=self.io)

        self.state = WorkflowState.IDLE
        self.current_step = 0

        # ROS2 서비스: 외부에서 워크플로우 트리거
        self.start_srv = self.create_service(
            Trigger, 'start_workflow', self._handle_start_workflow)

        self.get_logger().info('TaskManagerNode initialized.')
        self.get_logger().info('  Service: start_workflow (std_srvs/Trigger)')

    def _handle_start_workflow(self, request, response):
        """ROS2 서비스 핸들러: 외부에서 워크플로우 시작"""
        if self.state == WorkflowState.RUNNING:
            response.success = False
            response.message = 'Workflow already running.'
            return response

        self.object_manager.reset()
        success = self.run_workflow()
        response.success = success
        response.message = 'Workflow completed.' if success else 'Workflow failed.'
        return response

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {text}')

    def run_workflow(self) -> bool:
        """전체 워크플로우를 순서대로 실행"""
        # 로봇 초기화 (DSR_ROBOT2 import는 DR_init.__dsr__node 설정 이후에만 가능)
        self.io.set_tool_tcp()
        self.io.set_robot_mode_autonomous()

        # 기존 태스크 모듈의 IO 클라이언트 초기화
        from cobot1.main import setup_io_clients
        from cobot1 import press_test, source_test, powder_test
        setup_io_clients(self)
        press_test.setup_io_clients(self)
        source_test.setup_io_clients(self)
        powder_test.setup_io_clients(self)

        sauce = self.get_parameter('sauce').get_parameter_value().string_value
        powder = self.get_parameter('powder').get_parameter_value().string_value
        self.get_logger().info(f'Workflow start | sauce={sauce}, powder={powder}')

        self.state = WorkflowState.RUNNING

        steps = [
            ('1/6', '집게로 반죽 배치',    self._step_place_dough),
            ('2/6', '프레스로 반죽 누르기', self._step_press_dough),
            ('3/6', '접시 배치',           self._step_place_plate),
            ('4/6', '뒤집개로 뒤집기',     self._step_flip_item),
            ('5/6', '소스 뿌리기',         lambda: self._step_sauce(sauce)),
            ('6/6', '가루 뿌리기',         lambda: self._step_powder(powder)),
        ]

        for idx, (step_num, desc, step_fn) in enumerate(steps):
            self.current_step = idx + 1
            if self.state == WorkflowState.ERROR:
                break

            self.publish_status(f'작업 {step_num}: {desc}')

            success = self._execute_step(step_fn, desc)
            if not success:
                self.state = WorkflowState.ERROR
                self.publish_status(f'오류 발생: {desc}')
                return False

            self.get_logger().info(f'Step {step_num} completed: {desc}')

        self.state = WorkflowState.COMPLETED
        self.publish_status('완료 - 대기 중')
        return True

    def _execute_step(self, step_fn, desc: str) -> bool:
        try:
            result = step_fn()
            return result is not False
        except Exception as e:
            self.get_logger().error(f'Exception in [{desc}]: {e}')
            return False

    # ===== 개별 단계 =====

    def _step_place_dough(self) -> bool:
        self.tool_manager.pick_tool('tongs')
        self.dough_task.place_dough_with_tongs()
        self.tool_manager.return_tool('tongs')
        return True

    def _step_press_dough(self) -> bool:
        self.tool_manager.pick_tool('presser')
        self.press_task.press_dough()
        self.tool_manager.return_tool('presser')
        return True

    def _step_place_plate(self) -> bool:
        self.object_manager.pick_and_place_plate()
        return True

    def _step_flip_item(self) -> bool:
        self.tool_manager.pick_tool('spatula')
        self.flip_task.flip_item_with_spatula()
        self.tool_manager.return_tool('spatula')
        return True

    def _step_sauce(self, sauce: str) -> bool:
        if sauce == 'none':
            self.get_logger().info('Sauce skipped.')
            return True
        self.tool_manager.pick_tool('sauce_bottle')
        self.sauce_task.dispense_sauce()
        self.tool_manager.return_tool('sauce_bottle')
        return True

    def _step_powder(self, powder: str) -> bool:
        if powder == 'none':
            self.get_logger().info('Powder skipped.')
            return True
        self.tool_manager.pick_tool('powder_bottle')
        self.powder_task.sprinkle_powder()
        self.tool_manager.return_tool('powder_bottle')
        return True


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_CONFIG['robot_id']
    DR_init.__dsr__model = ROBOT_CONFIG['robot_model']

    node = TaskManagerNode()

    # DR_init 노드 연결 (main 스코프에서 설정해야 DSR_ROBOT2와 같은 DR_init 모듈 참조)
    DR_init.__dsr__node = node

    try:
        node.run_workflow()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    except Exception as e:
        import traceback
        node.get_logger().error(f'Unexpected error: {e}\n{traceback.format_exc()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
