#!/usr/bin/env python3
"""시뮬 정답 자세(nav_msgs/Odometry) → TF. rtabmap 이 맵 노드마다 정답 자세를 DB 에 남기게 한다.

rtabmap 은 정답을 TF 로만 받는다 (ground_truth_frame_id → ground_truth_base_frame_id 를 영상 시각에 조회해
Node.ground_truth_pose 에 저장). Isaac 은 정답을 Odometry 토픽으로만 낸다 (M1 /aeirobot/sim/gt_odom,
M2 /aeirobot/odometry_gt — 차체 시작 자세 기준). 이 노드가 그 사이를 잇는다.

로봇 TF 트리(map · odom · base_footprint)와 이어지지 않는 별도 두 프레임으로 낸다 — 메시지의 frame_id(odom) ·
child_frame_id(base_footprint)를 그대로 쓰면 EKF 가 내는 odom → base_footprint 와 겹친다.
aeirobot_rtabmap.launch.py 가 ground_truth_topic 인자를 받았을 때만 띄운다.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def to_transform(msg, frame_id, child_frame_id):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp               # 시뮬 시각 그대로 — rtabmap 이 영상 시각으로 보간해 찾는다
    t.header.frame_id, t.child_frame_id = frame_id, child_frame_id
    p = msg.pose.pose.position
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
    t.transform.rotation = msg.pose.pose.orientation
    return t


def newer(stamp, last):
    """이 시각의 TF 를 내도 되는가. 시각 0(시계가 돌기 전)과, 앞에 낸 것보다 늦지 않은 시각은 버린다 —
    같은 시각의 TF 를 두 번 내면 /tf 를 듣는 모든 노드(tf2)가 TF_REPEATED_DATA 경고를 찍는다."""
    return stamp != (0, 0) and (last is None or stamp > last)


def main():
    rclpy.init()
    node = rclpy.create_node('ground_truth_tf')
    frame = node.declare_parameter('frame_id', 'sim_ground_truth').value
    child = node.declare_parameter('child_frame_id', 'sim_ground_truth_base').value
    broadcaster = TransformBroadcaster(node)
    last = [None]

    def relay(msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if newer(stamp, last[0]):
            last[0] = stamp
            broadcaster.sendTransform(to_transform(msg, frame, child))

    def check():
        # 정답이 안 오는데 rtabmap 은 정답 프레임을 받았으면, 영상마다 wait_for_transform 만큼 기다리고 정답은 빈 채로 저장된다
        if last[0] is None:
            node.get_logger().warn(
                f"정답 자세가 5초째 안 온다 ({node.resolve_topic_name('ground_truth')}) — 시뮬이 이 토픽을 내는지 확인. "
                'rtabmap 은 정답 없이 저장하고 프레임마다 정답 TF 를 기다린다')
        timer.cancel()

    # best effort 구독은 Isaac 쪽이 reliable 이든 best effort 든 붙는다
    node.create_subscription(Odometry, 'ground_truth', relay, qos_profile_sensor_data)
    timer = node.create_timer(5.0, check, clock=Clock(clock_type=ClockType.STEADY_TIME))    # 시뮬 시계가 멈춰 있어도 알리게 벽시계로
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
