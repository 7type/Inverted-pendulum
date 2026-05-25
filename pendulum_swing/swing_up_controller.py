#!/usr/bin/env python3
"""
倒立摆起摆 + LQR 稳定控制器

能量起摆 → 三区域渐进制动 → LQR 稳定
控制周期 200Hz，切换带滞环防抖动
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class SwingUpController(Node):
    def __init__(self):
        super().__init__('swing_up_controller')

        # ROS2 接口
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/cart_effort_controller/commands',
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        # 200Hz 控制周期，太低了摆杆会抖，实时性也差
        self.timer = self.create_timer(0.005, self.control_loop)

        # 状态
        self.x = 0.0
        self.x_dot = 0.0
        self.theta_gazebo = 0.0   # 原始角度，保留多圈信息，算能量必须用这个
        self.theta = 0.0          # 归一化到 [0, 2π]，区域判断用
        self.theta_dot = 0.0
        self.state_ready = False

        # 物理参数
        self.g = 9.81
        self.M = 0.135
        self.m = 0.1
        self.l = 0.2
        self.L = 0.4
        self.I = (1.0 / 3.0) * self.m * (self.L ** 2)
        self.b = 0.2

        # 起摆
        self.k_energy = 25.0
        self.u_max = 45.0
        self.Er = 2.0 * self.m * self.g * self.l  # 倒立位置势能

        # 位置保护 PD
        self.kp_pos = 8.0
        self.kd_pos = 3.0

        # 速度软墙
        self.v_limit = 4.0

        # 边界
        self.x_boundary = 1.5
        self.x_hard_limit = 3.0

        # LQR 切换参数
        # 从 15°/3rad/s 放宽到 25°/5rad/s —— 之前条件太严，切换时角速度还很高，
        # 摆杆直接"跨过去"停不住，放宽后才稳
        # 滞环 25°进 / 45°出，防止在边界来回切
        self.switch_angle_in = np.deg2rad(25.0)
        self.switch_angle_out = np.deg2rad(45.0)
        self.switch_omega = 5.0
        self.switch_hold = 10        # 连续 10 个周期（50ms）满足条件才切，防瞬时误判
        self.switch_x_max = 4.0

        # Q 矩阵可调：角度权重 2000 是保竖直的最小的权重
        # 太高小车会抖，太低摆杆收敛慢
        #可等比调小，对K无影响
        self.Q = np.diag([100.0, 2000.0, 50.0, 100.0])
        self.R = np.array([[1.0]])

        self.K_lqr = -self._compute_lqr()

        # 状态机
        self.control_mode = 'swing_up'
        self.hold_count = 0
        self.last_dir_sign = 1.0

        # 日志
        self.log_counter = 0
        self.get_logger().info('=' * 60)
        self.get_logger().info('SwingUp + LQR Controller Started')
        self.get_logger().info(f'Physical: M={self.M}, m={self.m}, l={self.l}, I={self.I:.6f}')
        self.get_logger().info(f'LQR K = [{self.K_lqr[0]:.2f}, {self.K_lqr[1]:.2f}, {self.K_lqr[2]:.2f}, {self.K_lqr[3]:.2f}]')
        self.get_logger().info(f'Target Energy Er = {self.Er:.4f} J')
        self.get_logger().info(f'Switch: IN<{np.degrees(self.switch_angle_in):.0f}° OUT>{np.degrees(self.switch_angle_out):.0f}° |x|<{self.switch_x_max}m')
        self.get_logger().info('=' * 60)

    def _compute_lqr(self):
        M, m, l, I, g = self.M, self.m, self.l, self.I, self.g
        Mt = M + m
        b_cart = self.b
        b_pole = 0.00007892   # Gazebo 里 pole_joint 的阻尼，仿真和现实的差距得补上

        denom = Mt * (m * l**2 + I) - (m * l)**2

        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0,  (m**2 * l**2 * g) / denom, -(m*l**2 + I)*b_cart / denom, -m*l*b_pole / denom],
            [0.0,  (Mt * m * l * g) / denom, -m*l*b_cart / denom, -Mt*b_pole / denom]
        ])

        B = np.array([
            [0.0],
            [0.0],
            [(m * l**2 + I) / denom],
            [-m * l / denom]   # 这个负号很关键，匹配 Gazebo 坐标系，否则力的方向正好相反
        ])

        P = self._solve_care_hamilton(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ B.T @ P
        return K.flatten()

    @staticmethod
    def _solve_care_hamilton(A, B, Q, R):
      #  Hamilton 矩阵法解CARE，不依赖 scipy，纯 NumPy 实现
        n = A.shape[0]
        R_inv = np.linalg.inv(R)
        H = np.block([
            [A, -B @ R_inv @ B.T],
            [-Q, -A.T]
        ])
        eigvals, eigvecs = np.linalg.eig(H)
        stable_idx = np.where(np.real(eigvals) < 0)[0]
        if len(stable_idx) != n:
            raise ValueError(
                f"CARE solve failed: expected {n} stable eigenvalues, got {len(stable_idx)}. "
                f"Eigenvalues: {eigvals}"
            )
        V = eigvecs[:, stable_idx]
        X1 = V[:n, :]
        X2 = V[n:, :]
        P = np.real(X2 @ np.linalg.inv(X1))
        return P

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'cart_joint':
                self.x = msg.position[i]
                self.x_dot = msg.velocity[i]
            elif name == 'pole_joint':
                self.theta_gazebo = msg.position[i]
                self.theta = self.theta_gazebo % (2.0 * np.pi)
                self.theta_dot = msg.velocity[i]
        self.state_ready = True

        if not hasattr(self, '_logged_init'):
            self._logged_init = True
            self.get_logger().info(
                f'[INIT] theta_gazebo={self.theta_gazebo:.4f} rad '
                f'theta_norm={np.degrees(self.theta):.1f}°, x={self.x:.4f} m'
            )

    def compute_energy(self):
        E_k = 0.5 * self.I * (self.theta_dot ** 2)
        E_p = self.m * self.g * self.l * (1.0 - np.cos(self.theta_gazebo))
        return E_k + E_p

    def swing_up_control(self):
        E = self.compute_energy()
        E_err = E - self.Er

        # 相位检测：θ̇·cos(θ) 决定推力方向
        # 最低点死区处理：角速度接近 0 时给默认方向，避免换向抖动
        phase = self.theta_dot * np.cos(self.theta)
        if abs(phase) < 1e-6:
            at_bottom = (self.theta < 0.5) or (self.theta > 2 * np.pi - 0.5)
            if abs(self.theta_dot) < 0.3 and at_bottom:
                dir_sign = 1.0
            else:
                dir_sign = self.last_dir_sign
        else:
            dir_sign = np.sign(phase)
            self.last_dir_sign = dir_sign

        theta_err = np.arctan2(np.sin(self.theta - np.pi), np.cos(self.theta - np.pi))
        angle_to_up = abs(theta_err)

        # 三区域：下半区能量注入 / 过渡区渐进制动 / 捕获区 LQR 接管
        # 单一起摆律搞不定，必须分区 —— 下半区要猛加能量，上半区要减速，捕获区要精细控制
        is_lower_half = angle_to_up > np.deg2rad(90.0)
        is_capture_zone = angle_to_up < np.deg2rad(25.0)
        is_transition = (not is_lower_half) and (not is_capture_zone)

        # ========== 捕获区 ==========
        if is_capture_zone:
            if (angle_to_up < self.switch_angle_in and
                abs(self.theta_dot) < self.switch_omega and
                abs(self.x) < self.switch_x_max):
                force = self.balance_control()
                debug_info = 'LQR_TAKEOVER'

            elif abs(self.theta_dot) > 12.0:
                # 角速度太高，中等制动 + 位置回中
                brake_force = min(15.0, abs(self.theta_dot) * 1.2)
                force = -np.sign(self.theta_dot) * brake_force
                force += -2.0 * self.x - 0.5 * self.x_dot
                debug_info = f'ULTRA_BRAKE={brake_force:.1f}'

            elif abs(self.theta_dot) > 6.0:
                # 温和制动
                brake_force = abs(self.theta_dot) * 0.8
                force = -np.sign(self.theta_dot) * brake_force
                force += -3.0 * self.x - 1.0 * self.x_dot
                debug_info = f'brake={brake_force:.1f}'

            else:
                # 角速度可控，PD 纠正角度 + 阻尼 + 位置回中
                f_angle = -30.0 * theta_err
                f_damp = -20.0 * self.theta_dot
                f_pos = -10.0 * self.x
                f_vel = -5.0 * self.x_dot
                force = f_angle + f_damp + f_pos + f_vel
                debug_info = f'ang={f_angle:+.1f} damp={f_damp:+.1f} pos={f_pos:+.1f} vel={f_vel:+.1f}'

            # 速度软墙
            if abs(self.x_dot) > self.v_limit:
                v_excess = abs(self.x_dot) - self.v_limit
                f_vwall = -np.sign(self.x_dot) * v_excess * 10.0
                force += f_vwall
                debug_info += f' vwall={f_vwall:+.1f}'

            # 硬限位兜底
            if abs(self.x) > self.x_hard_limit:
                force = -35.0 * np.sign(self.x)
                debug_info = 'HARD_LIMIT'

            force = float(np.clip(force, -40.0, 40.0))

            self.log_counter += 1
            if self.log_counter % 200 == 0:
                self.get_logger().info(
                    f'[CAPTURE] th={np.degrees(self.theta):.1f}° '
                    f'thd={self.theta_dot:+.2f} x={self.x:+.3f} xd={self.x_dot:+.2f} '
                    f'F={force:+.1f}N [{debug_info}]'
                )
            return force

        # ========== 过渡区 ==========
        if is_transition:
            if abs(self.x) > 1.5:
                # 小车快跑偏了，优先回中，摆杆先不管
                force = -self.kp_pos * self.x - self.kd_pos * self.x_dot
                force += -0.5 * self.theta_dot
                debug_info = 'rescue_pos'

            elif abs(self.theta_dot) > 8.0:
                brake_force = min(12.0, abs(self.theta_dot) * 0.8)
                force = -np.sign(self.theta_dot) * brake_force
                force += -self.kp_pos * 0.5 * self.x - self.kd_pos * 0.5 * self.x_dot
                debug_info = f'brake={brake_force:.1f}'

            elif abs(self.theta_dot) > 4.0:
                brake_force = abs(self.theta_dot) * 0.6
                force = -np.sign(self.theta_dot) * brake_force
                force += -self.kp_pos * 0.3 * self.x - self.kd_pos * 0.3 * self.x_dot
                debug_info = f'soft_brake={brake_force:.1f}'

            else:
                force = -self.kp_pos * 0.3 * self.x - self.kd_pos * 0.3 * self.x_dot
                force += -0.8 * self.theta_dot
                debug_info = 'glide'

            if abs(self.x_dot) > self.v_limit:
                v_excess = abs(self.x_dot) - self.v_limit
                f_vwall = -np.sign(self.x_dot) * v_excess * 4.0
                force += f_vwall
                debug_info += f' vwall={f_vwall:+.1f}'

            if abs(self.x) > self.x_hard_limit:
                force = -35.0 * np.sign(self.x)
                debug_info = 'HARD_LIMIT'

            force = float(np.clip(force, -self.u_max, self.u_max))

            self.log_counter += 1
            if self.log_counter % 200 == 0:
                self.get_logger().info(
                    f'[TRANS] E={E:.3f}J F={force:+.1f}N '
                    f'th={np.degrees(self.theta):.1f}° thd={self.theta_dot:+.2f} x={self.x:+.3f} '
                    f'[{debug_info}]'
                )
            return force

        # ========== 下半区：能量注入 ==========
        # 角速度失控保护：超过 15 rad/s 强制阻尼，位置保护让路
        if abs(self.theta_dot) > 15.0:
            force_energy = -np.sign(self.theta_dot) * min(self.u_max * 0.5, abs(self.theta_dot) * 1.2)
            pos_weight = 0.0
            debug_info = f'omega_brake={force_energy:+.1f}'
        else:
            if E_err < -0.03:
                force_energy = +dir_sign * min(self.u_max * 0.5, abs(E_err) * self.k_energy)
            elif E_err > 0.03:
                # 能量过冲用固定 6N 制动，不能跟 E_err 成正比
                # 之前试过比例制动，误差越大制动力越大，反而把摆杆往回拉，能量泄漏
                force_energy = -dir_sign * 6.0
            else:
                force_energy = 0.0
            debug_info = f'energy={force_energy:+.1f}'

            # 位置保护权重随能量自适应
            # 能量低时几乎不管位置（0.05），能量接近目标时逐渐收紧
            if E > self.Er * 1.5:
                pos_weight = 0.0
            elif E < self.Er * 0.6:
                pos_weight = 0.05
            elif E < self.Er * 0.9:
                pos_weight = 0.15
            else:
                pos_weight = min(0.25, abs(self.x) / 2.0)

        pos_force = -self.kp_pos * self.x - self.kd_pos * self.x_dot

        if self.log_counter % 50 == 0:
            self.get_logger().info(
                f'[PHYSICS] th={np.degrees(self.theta):5.1f}° '
                f'thd={self.theta_dot:+.2f} '
                f'phase={phase:+.2f} dir_sign={dir_sign:+.0f} '
                f'E_err={E_err:+.3f}J '
                f'force_energy_raw={force_energy:+.1f}N '
                f'pos_w={pos_weight:.2f} pos_f={pos_force:+.1f}N'
            )

        force = (1.0 - pos_weight) * force_energy + pos_weight * pos_force
        debug_info += f' w={pos_weight:.2f} pos_f={pos_force:+.1f}'

        if abs(self.x_dot) > self.v_limit:
            v_excess = abs(self.x_dot) - self.v_limit
            f_vwall = -np.sign(self.x_dot) * v_excess * 6.0
            force += f_vwall
            debug_info += f' vwall={f_vwall:+.1f}'

        if abs(self.x) > self.x_hard_limit:
            force = -35.0 * np.sign(self.x)
            debug_info = 'HARD_LIMIT'

        force = float(np.clip(force, -self.u_max, self.u_max))

        self.log_counter += 1
        if self.log_counter % 200 == 0:
            self.get_logger().info(
                f'[SWING] E={E:.3f}J err={E_err:+.3f}J '
                f'w_pos={pos_weight:.2f} F={force:+.1f}N '
                f'th={np.degrees(self.theta):.1f}° thd={self.theta_dot:+.2f} x={self.x:+.3f} '
                f'[{debug_info}]'
            )

        return force

    def balance_control(self):
        theta_err = np.arctan2(np.sin(self.theta - np.pi), np.cos(self.theta - np.pi))
        state_err = np.array([self.x, theta_err, self.x_dot, self.theta_dot])
        force = np.dot(self.K_lqr, state_err)
        return float(np.clip(force, -20.0, 20.0))

    def control_loop(self):
        if not self.state_ready:
            return

        theta_err = np.arctan2(np.sin(self.theta - np.pi), np.cos(self.theta - np.pi))
        angle_to_up = abs(theta_err)

        if self.control_mode == 'swing_up':
            if (angle_to_up < self.switch_angle_in and
                abs(self.theta_dot) < self.switch_omega and
                abs(self.x) < self.switch_x_max):
                self.hold_count += 1
                if self.hold_count >= self.switch_hold:
                    self.control_mode = 'balance'
                    self.hold_count = 0
                    self.get_logger().info(
                        f'[SWITCH] BALANCE: |th-pi|={np.degrees(angle_to_up):.1f}° '
                        f'thd={self.theta_dot:+.3f} x={self.x:+.3f}'
                    )
            else:
                self.hold_count = 0
        else:
            if angle_to_up > self.switch_angle_out:
                self.control_mode = 'swing_up'
                self.hold_count = 0
                self.get_logger().warn(
                    f'[SWITCH] SWING_UP: |th-pi|={np.degrees(angle_to_up):.1f}° '
                    f'thd={self.theta_dot:+.3f} x={self.x:+.3f}'
                )

        if self.control_mode == 'swing_up':
            force = self.swing_up_control()
        else:
            force = self.balance_control()

        msg = Float64MultiArray()
        msg.data = [force]
        self.effort_pub.publish(msg)

        if self.log_counter % 200 == 0:
            E = self.compute_energy()
            self.get_logger().info(
                f'[{self.control_mode:7s}] th={np.degrees(self.theta):6.1f}° '
                f'thd={self.theta_dot:+.3f} x={self.x:+.3f} E={E:.3f}J F={force:+.2f}N'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SwingUpController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
