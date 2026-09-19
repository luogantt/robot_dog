#!/usr/bin/env python3
"""Fail-closed command arbiter and the sole publisher of the S10 /STEER topic."""

import math
import time

from drdds.msg import Steer
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt8
from std_srvs.srv import SetBool, Trigger

from .asdu_link import AsduLink
from .common import approach, clamp
from .obstacle_guard import AVOID, CLEAR, SLOW, STOP

# 输出通道：
#   'dds'  —— 只发 /STEER（原方案，需要 rl_deploy 订阅）
#   'asdu' —— 只发 ASDU UDP 轴指令（直连机器人本体，不需要 rl_deploy）
#   'both' —— 两路都发。⚠️ 仅在确认只有一个接收端生效时使用，
#             否则两个运控同时控制会打架（ASDU 侧还有 2 秒同源约束）
VALID_CHANNELS = ('dds', 'asdu', 'both')


class CommandMux(Node):
    def __init__(self) -> None:
        super().__init__('s10_command_mux')
        self.declare_parameter('output_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('feed_timeout_sec', 0.5)
        self.declare_parameter('max_vx_mps', 0.80)
        self.declare_parameter('max_vy_mps', 0.20)
        self.declare_parameter('max_wz_rps', 0.50)
        self.declare_parameter('max_linear_accel_mps2', 0.60)
        self.declare_parameter('max_lateral_accel_mps2', 0.40)
        self.declare_parameter('max_angular_accel_rps2', 0.80)
        self.declare_parameter('steer_vx_scale_mps', 1.50)
        self.declare_parameter('steer_vy_scale_mps', 0.50)
        self.declare_parameter('steer_wz_scale_rps', 0.60)
        # 输出通道配置
        self.declare_parameter('output_channel', 'dds')
        self.declare_parameter('asdu_host', '10.21.33.103')
        self.declare_parameter('asdu_port', 30004)
        self.declare_parameter('asdu_heartbeat_hz', 1.0)
        # 机器人状态门（仅 asdu 通道生效）
        self.declare_parameter('require_rl_control', True)
        self.declare_parameter('robot_status_timeout_sec', 1.0)

        parameter = lambda name: self.get_parameter(name).value
        self.rate = max(1.0, float(parameter('output_rate_hz')))
        self.timeout = float(parameter('command_timeout_sec'))
        self.feed_timeout = float(parameter('feed_timeout_sec'))
        self.max_vx = float(parameter('max_vx_mps'))
        self.max_vy = float(parameter('max_vy_mps'))
        self.max_wz = float(parameter('max_wz_rps'))
        self.max_ax = float(parameter('max_linear_accel_mps2'))
        self.max_ay = float(parameter('max_lateral_accel_mps2'))
        self.max_aw = float(parameter('max_angular_accel_rps2'))
        self.steer_vx_scale = float(parameter('steer_vx_scale_mps'))
        self.steer_vy_scale = float(parameter('steer_vy_scale_mps'))
        self.steer_wz_scale = float(parameter('steer_wz_scale_rps'))

        self.output_channel = str(parameter('output_channel')).lower()
        if self.output_channel not in VALID_CHANNELS:
            raise ValueError(
                f"output_channel must be one of {VALID_CHANNELS}, "
                f"got {self.output_channel!r}")
        self.asdu = None
        self.require_rl_control = bool(parameter('require_rl_control'))
        self.robot_status_timeout = float(parameter('robot_status_timeout_sec'))
        if self.output_channel in ('asdu', 'both'):
            self.asdu = AsduLink(parameter('asdu_host'), parameter('asdu_port'),
                                 heartbeat_hz=float(parameter('asdu_heartbeat_hz')))
        self._last_asdu_error = 0.0
        self._last_robot_log = 0.0

        self.tracking = Twist()
        self.avoidance = Twist()
        self.last_tracking = None
        self.last_avoidance = None
        self.health = False
        self.last_health = None
        self.obstacle_state = STOP
        self.last_obstacle_state = None
        self.obstacle_scale = 0.0
        self.enabled = False
        self.emergency_latched = False
        self.finished = False
        self.output = Twist()
        self.frame_counter = 0
        self.last_state_text = ''

        self.create_subscription(
            TwistStamped, '/s10_nav/cmd_tracking', self.on_tracking, 10)
        self.create_subscription(
            TwistStamped, '/s10_nav/cmd_avoidance', self.on_avoidance, 10)
        self.create_subscription(UInt8, '/s10_nav/obstacle_state',
                                 self.on_obstacle_state, 10)
        self.create_subscription(Float32, '/s10_nav/obstacle_speed_scale',
                                 lambda msg: setattr(self, 'obstacle_scale',
                                                     clamp(float(msg.data), 0.0, 1.0)), 10)
        self.create_subscription(Bool, '/s10_nav/healthy', self.on_health, 10)
        self.create_subscription(Bool, '/s10_nav/finished', self.on_finished, 10)
        self.steer_pub = self.create_publisher(Steer, '/STEER', 10)
        self.state_pub = self.create_publisher(String, '/s10_nav/state', 10)
        self.create_service(SetBool, '/s10_nav/set_enabled', self.set_enabled)
        self.create_service(Trigger, '/s10_nav/emergency_stop', self.emergency_stop)
        self.create_service(Trigger, '/s10_nav/reset_emergency', self.reset_emergency)
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        if self.asdu is not None:
            self.get_logger().warning(
                f'output_channel={self.output_channel}: 轴指令将发往 '
                f'ASDU {self.asdu.host}:{self.asdu.port}（不经过 rl_deploy）。'
                f' 注意：该指令无响应帧，且需机器人处于 RL 控制状态。')
        else:
            self.get_logger().info(
                f'output_channel={self.output_channel}: 轴指令只发 /STEER。')

    def on_tracking(self, message: TwistStamped) -> None:
        self.tracking = message.twist
        self.last_tracking = time.monotonic()

    def on_avoidance(self, message: TwistStamped) -> None:
        self.avoidance = message.twist
        self.last_avoidance = time.monotonic()

    def on_obstacle_state(self, message: UInt8) -> None:
        self.obstacle_state = int(message.data)
        self.last_obstacle_state = time.monotonic()

    def on_health(self, message: Bool) -> None:
        new_health = bool(message.data)
        if self.health and not new_health and self.enabled:
            self.enabled = False
            self.get_logger().error('health lost: autonomy disabled; explicit re-enable required')
        self.health = new_health
        self.last_health = time.monotonic()

    def feed_is_stale(self) -> bool:
        """True when /s10_nav/healthy or /s10_nav/obstacle_state stopped updating.

        A dead health_monitor or obstacle_guard would otherwise leave the mux
        acting on their last published (possibly nominal) values forever.
        """
        now = time.monotonic()
        if self.last_health is None or now - self.last_health > self.feed_timeout:
            return True
        if self.last_obstacle_state is None or now - self.last_obstacle_state > self.feed_timeout:
            return True
        return False

    def on_finished(self, message: Bool) -> None:
        self.finished = bool(message.data)
        if self.finished:
            self.enabled = False

    def set_enabled(self, request: SetBool.Request,
                    response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if not self.health or self.feed_is_stale():
                response.success = False
                response.message = 'cannot enable: health gate is not ready'
                return response
            if self.emergency_latched:
                response.success = False
                response.message = 'cannot enable: emergency stop is latched'
                return response
            if self.finished:
                response.success = False
                response.message = 'cannot enable: route is already finished'
                return response
        self.enabled = bool(request.data)
        response.success = True
        response.message = 'autonomy enabled' if self.enabled else 'autonomy disabled'
        self.get_logger().warning(response.message)
        return response

    def emergency_stop(self, _request: Trigger.Request,
                       response: Trigger.Response) -> Trigger.Response:
        self.emergency_latched = True
        self.enabled = False
        response.success = True
        response.message = 'emergency stop latched'
        self.get_logger().error(response.message)
        return response

    def reset_emergency(self, _request: Trigger.Request,
                        response: Trigger.Response) -> Trigger.Response:
        if not self.health:
            response.success = False
            response.message = 'cannot reset emergency while health gate is not ready'
            return response
        self.emergency_latched = False
        response.success = True
        response.message = 'emergency latch reset; autonomy remains disabled'
        self.get_logger().warning(response.message)
        return response

    @staticmethod
    def command_is_finite(command: Twist) -> bool:
        values = [command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z]
        return all(math.isfinite(value) for value in values)

    def select_target(self) -> tuple:
        now = time.monotonic()
        if self.emergency_latched:
            return Twist(), 'EMERGENCY'
        if not self.enabled:
            return Twist(), 'DISABLED'
        if self.feed_is_stale():
            self.enabled = False
            return Twist(), 'FEED_TIMEOUT'
        if not self.health:
            self.enabled = False
            return Twist(), 'HEALTH_FAULT'
        if self.asdu is not None and self.require_rl_control:
            # 轴指令只在 RL 控制状态下有效（§1.2.3 / §2.2.1），而且**没有响应帧**——
            # 不在 RL 控制时它会被机器人静默忽略。没有这道门，导航栈会以为在走、
            # 实际机器人一动不动。状态靠心跳触发的主动上报获得。
            if self.asdu.status_age() > self.robot_status_timeout:
                self.enabled = False
                return Twist(), 'ROBOT_STATUS_LOST'
            if not self.asdu.is_rl_control():
                self.enabled = False
                return Twist(), 'ROBOT_NOT_RL'
        if self.finished:
            self.enabled = False
            return Twist(), 'FINISHED'
        if self.last_tracking is None or now - self.last_tracking > self.timeout:
            self.enabled = False
            return Twist(), 'TRACKING_TIMEOUT'
        if not self.command_is_finite(self.tracking):
            self.enabled = False
            return Twist(), 'INVALID_TRACKING_COMMAND'
        if self.obstacle_state == STOP:
            return Twist(), 'OBSTACLE_STOP'
        if self.obstacle_state == AVOID:
            if self.last_avoidance is None or now - self.last_avoidance > self.timeout:
                return Twist(), 'AVOIDANCE_TIMEOUT'
            if not self.command_is_finite(self.avoidance):
                return Twist(), 'INVALID_AVOIDANCE_COMMAND'
            return self.avoidance, 'AVOIDING'

        target = Twist()
        scale = self.obstacle_scale if self.obstacle_state == SLOW else 1.0
        target.linear.x = self.tracking.linear.x * scale
        target.linear.y = self.tracking.linear.y * scale
        target.angular.z = self.tracking.angular.z
        return target, 'SLOW' if self.obstacle_state == SLOW else 'RUNNING'

    def limited_command(self, target: Twist, immediate_stop: bool) -> Twist:
        command = Twist()
        if immediate_stop:
            return command
        dt = 1.0 / self.rate
        target_x = clamp(float(target.linear.x), -self.max_vx, self.max_vx)
        target_y = clamp(float(target.linear.y), -self.max_vy, self.max_vy)
        target_w = clamp(float(target.angular.z), -self.max_wz, self.max_wz)
        command.linear.x = approach(self.output.linear.x, target_x, self.max_ax * dt)
        command.linear.y = approach(self.output.linear.y, target_y, self.max_ay * dt)
        command.angular.z = approach(self.output.angular.z, target_w, self.max_aw * dt)
        return command

    def publish_steer(self, command: Twist) -> None:
        # 归一化只算一次，两条通道共用同一组数值。
        # 依据：官方 SDK 里 /STEER 的 x 和 ASDU 的 X 写进的是同一个 usr_cmd_ 字段，
        # 标度一致，所以不能各自再缩放一次。
        steer_x = float(clamp(command.linear.x / self.steer_vx_scale, -1.0, 1.0))
        steer_y = float(clamp(command.linear.y / self.steer_vy_scale, -1.0, 1.0))
        steer_yaw = float(clamp(command.angular.z / self.steer_wz_scale, -1.0, 1.0))

        if self.output_channel in ('dds', 'both'):
            message = Steer()
            message.header.frame_id = self.frame_counter
            message.header.stamp = self.get_clock().now().to_msg()
            message.data.x = steer_x
            message.data.y = steer_y
            message.data.z = 0.0
            message.data.roll = 0.0
            message.data.pitch = 0.0
            message.data.yaw = steer_yaw
            self.steer_pub.publish(message)

        if self.asdu is not None:
            if not self.asdu.send_axis(steer_x, steer_y, steer_yaw):
                # 发送失败要看得见，但别在 50Hz 循环里刷屏
                now = time.monotonic()
                if now - self._last_asdu_error > 2.0:
                    self._last_asdu_error = now
                    self.get_logger().error(
                        f'ASDU send failed ({self.asdu.send_failures} total)')
            self.asdu.maybe_heartbeat()

        self.frame_counter = (self.frame_counter + 1) % (2 ** 64)

    def destroy_node(self) -> bool:
        if self.asdu is not None:
            self.asdu.close()
        return super().destroy_node()

    def update(self) -> None:
        if self.asdu is not None:
            self.asdu.poll()          # 先收状态，再做门控判断
        target, state = self.select_target()
        immediate_stop = state not in ('RUNNING', 'SLOW', 'AVOIDING')
        self.output = self.limited_command(target, immediate_stop)
        self.publish_steer(self.output)
        if state != self.last_state_text:
            self.state_pub.publish(String(data=state))
            detail = ''
            if self.asdu is not None:
                detail = (f' [ASDU {self.asdu.host}:{self.asdu.port} '
                          f'sent={self.asdu.axis_sent} beat={self.asdu.heartbeats_sent} '
                          f'| {self.asdu.describe()}]')
            self.get_logger().info(f'navigation state: {state}{detail}')
            self.last_state_text = state

        # 每 5 秒打一次机器人状态，方便现场判断"为什么不动"
        if self.asdu is not None:
            now = time.monotonic()
            if now - self._last_robot_log > 5.0:
                self._last_robot_log = now
                self.get_logger().info(f'robot: {self.asdu.describe()}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
