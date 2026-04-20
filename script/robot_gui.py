import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# dsr_msgs2 패키지 서비스들
from dsr_msgs2.srv import GetRobotState, GetCurrentPosx, GetJointTorque, SetRobotMode
import tkinter as tk

class RobotGui(tk.Tk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.title("Doosan M0609 Full Monitor & Control")
        self.geometry("650x850")
        self.configure(bg='#f0f0f0')

        # 1. 로봇 제어 섹션
        self.frame_ctrl = tk.LabelFrame(self, text="Robot Control", padx=10, pady=10, font=("Arial", 11, "bold"))
        self.frame_ctrl.pack(fill="x", padx=15, pady=10)

        self.btn_servo_on = tk.Button(self.frame_ctrl, text="SERVO ON", bg="#28a745", fg="white", width=15,
                                      command=lambda: self.node.call_set_mode(1))
        self.btn_servo_on.pack(side="left", padx=10)

        self.btn_servo_off = tk.Button(self.frame_ctrl, text="SERVO OFF", bg="#dc3545", fg="white", width=15,
                                       command=lambda: self.node.call_set_mode(0))
        self.btn_servo_off.pack(side="left", padx=10)

        # 2. 시스템 상태
        self.lbl_state = tk.Label(self, text="State: Disconnected", font=("Arial", 14, "bold"), fg="blue")
        self.lbl_state.pack(pady=10)

        # 3. TCP 좌표 (X, Y, Z, RX, RY, RZ)
        self.frame_tcp = tk.LabelFrame(self, text="TCP Pose (Cartesian)", padx=10, pady=10, font=("Arial", 11, "bold"))
        self.frame_tcp.pack(fill="x", padx=15, pady=5)
        
        self.tcp_labels = []
        names = ['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (deg)', 'RY (deg)', 'RZ (deg)']
        for name in names:
            lbl = tk.Label(self.frame_tcp, text=f"{name}: 0.00", font=("Courier", 12))
            lbl.pack(anchor="w")
            self.tcp_labels.append(lbl)

        # 4. 관절 데이터 (Angle & Torque)
        self.frame_joints = tk.LabelFrame(self, text="Joint Data (Angle | Torque)", padx=10, pady=10, font=("Arial", 11, "bold"))
        self.frame_joints.pack(fill="x", padx=15, pady=5)
        
        self.joint_rows = []
        for i in range(6):
            lbl = tk.Label(self.frame_joints, text=f"J{i+1}: 0.00° | 0.00 Nm", font=("Courier", 12))
            lbl.pack(anchor="w")
            self.joint_rows.append(lbl)

    def update_display(self, angles, torques, tcp, state_str):
        # 상태 업데이트
        self.lbl_state.config(text=f"Robot State: {state_str}")
        
        # TCP 업데이트 (X, Y, Z, RX, RY, RZ)
        for i in range(6):
            val = tcp[i] if i < len(tcp) else 0.0
            self.tcp_labels[i].config(text=f"{['X','Y','Z','RX','RY','RZ'][i]}: {val:10.2f}")
        
        # 관절 각도 및 토크 업데이트
        for i in range(6):
            ang = angles[i] * (180.0 / 3.14159) if i < len(angles) else 0.0
            trq = torques[i] if i < len(torques) else 0.0
            self.joint_rows[i].config(text=f"J{i+1}: {ang:8.2f}° | {trq:8.2f} Nm")

class MonitorNode(Node):
    def __init__(self):
        super().__init__('m0609_total_monitor')
        self.angles = [0.0]*6
        self.torques = [0.0]*6
        self.tcp = [0.0]*6
        self.state = "Connecting..."

        # Subscriber
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_cb, 10)
        
        # Clients
        self.cli_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')
        self.cli_posx = self.create_client(GetCurrentPosx, '/dsr01/aux_control/get_current_posx')
        self.cli_torque = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        self.cli_mode = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')

        self.create_timer(0.5, self.update_loop)

    def joint_cb(self, msg):
        self.angles = msg.position

    def call_set_mode(self, mode):
        if self.cli_mode.service_is_ready():
            req = SetRobotMode.Request()
            req.robot_mode = mode
            self.cli_mode.call_async(req)

    def update_loop(self):
        # 서비스들 호출
        if self.cli_state.service_is_ready():
            self.cli_state.call_async(GetRobotState.Request()).add_done_callback(self.state_cb)
        if self.cli_posx.service_is_ready():
            self.cli_posx.call_async(GetCurrentPosx.Request()).add_done_callback(self.posx_cb)
        if self.cli_torque.service_is_ready():
            self.cli_torque.call_async(GetJointTorque.Request()).add_done_callback(self.torque_cb)

    def state_cb(self, future):
        try: self.state = str(future.result().robot_state)
        except: pass

    def posx_cb(self, future):
        try: self.tcp = future.result().task_pos
        except: pass

    def torque_cb(self, future):
        try: self.torques = future.result().jts
        except: pass

def main():
    rclpy.init()
    node = MonitorNode()
    app = RobotGui(node)

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            app.update_display(node.angles, node.torques, node.tcp, node.state)

    threading.Thread(target=ros_spin, daemon=True).start()
    app.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()