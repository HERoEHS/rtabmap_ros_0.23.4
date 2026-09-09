#
# lifelong SLAM 검증용 bag 녹화 — 카메라 무관 (camera:=zed|orbbec, 기본 env AEIROBOT_CAMERA)
#
# Khronos/rtabmap 재생에 필요한 토픽만 기록한다. rtabmap SLAM/viz 는 녹화에 불필요해서 안 켠다.
# aeirobot_rtabmap.launch.py 와 같은 규칙: 카메라 프리셋이 토픽·드라이버를, odom 은 기본 외부
# EKF(/odometry/filtered), VO 는 launch_odometry:=true 일 때만 (핸드헬드 — EKF 와 동시 금지).
#
# 사용법:
#   ros2 launch rtabmap_launch aeirobot_record.launch.py                       # 로봇: EKF odom 기록
#   ros2 launch rtabmap_launch aeirobot_record.launch.py camera:=orbbec launch_odometry:=true  # 핸드헬드 Gemini
#   ros2 launch rtabmap_launch aeirobot_record.launch.py bag_name:=office_test2
#   (끝낼 때 Ctrl-C — bag 이 자동 마무리됨)
#
# 저장 위치: ~/datasets/lifelong/<bag_name>_<YYYYmmdd_HHMMSS>/
# 기록: rgb, rgb camera_info, depth, (orbbec+use_imu: /camera/gyro_accel/sample), odom, /tf, /tf_static
# 640x480@15fps 기준 10분 ≈ 14GB.
#
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BAG_ROOT = os.path.expanduser('~/datasets/lifelong')
STAMP = datetime.now().strftime('%Y%m%d_%H%M%S')

# 토픽 프리셋 — aeirobot_rtabmap.launch.py _CAM_PRESETS 와 같은 값 (launch 는 모듈로 못 끌어와 복사)
_CAM_PRESETS = {
    'orbbec': {'rgb': '/camera/color/image_raw', 'info': '/camera/color/camera_info',
               'depth': '/camera/depth/image_raw', 'imu_raw': '/camera/gyro_accel/sample', 'driver': True},
    'zed':    {'rgb': '/aeirobot/vslam_left_image', 'info': '/aeirobot/vslam_left_camera_info',
               'depth': '/aeirobot/vslam_depth', 'imu_raw': None, 'driver': False},
}


def _setup(context, *_):
    cam = LaunchConfiguration('camera').perform(context)
    if cam not in _CAM_PRESETS:
        raise RuntimeError(f"[aeirobot_record] camera:={cam!r} — 가능한 값: {', '.join(_CAM_PRESETS)}")
    p = _CAM_PRESETS[cam]
    vo = LaunchConfiguration('launch_odometry').perform(context).lower() in ('true', '1')
    use_imu = vo and p['imu_raw'] is not None      # VO 중력 정렬용 — 카메라 IMU 가 있을 때만
    odom_topic = '/rtabmap/odom' if vo else '/odometry/filtered'
    topics = [p['rgb'], p['info'], p['depth'], odom_topic, '/tf', '/tf_static']
    if use_imu:
        topics.append(p['imu_raw'])
    acts = []
    if p['driver']:
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rtabmap_launch'), 'launch', 'gemini_camera.launch.py')),
            launch_arguments={'use_imu': 'true' if use_imu else 'false'}.items()))
    if vo:
        print('[aeirobot_record] VO 모드: rgbd_odometry 가 odom→base TF 를 발행 — EKF 와 동시 기동 금지.')
        if use_imu:
            acts.append(Node(
                package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
                output='screen',
                parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
                remappings=[('imu/data_raw', p['imu_raw']), ('imu/data', '/rtabmap/imu')]))
        acts.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
            namespace='rtabmap',
            parameters=[{'frame_id': 'base_footprint', 'approx_sync': True,
                         'wait_imu_to_init': use_imu, 'qos': 2, 'qos_camera_info': 2}],
            remappings=[('rgb/image', p['rgb']), ('rgb/camera_info', p['info']),
                        ('depth/image', p['depth']), ('imu', '/rtabmap/imu')]))
    if LaunchConfiguration('monitor').perform(context).lower() in ('true', '1'):
        acts.append(Node(package='rqt_image_view', executable='rqt_image_view', name='record_monitor',
                         arguments=[p['rgb']]))
    os.makedirs(BAG_ROOT, exist_ok=True)
    bag = LaunchConfiguration('bag_name').perform(context) + '_' + STAMP
    acts.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag] + topics, cwd=BAG_ROOT, output='screen'))
    acts.append(LogInfo(msg=f'녹화 시작 [{cam}, odom={odom_topic}]: {os.path.join(BAG_ROOT, bag)}  (종료: Ctrl-C)'))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value=os.environ.get('AEIROBOT_CAMERA', 'zed'),
                              description='zed | orbbec. 기본 = env AEIROBOT_CAMERA, 없으면 zed'),
        DeclareLaunchArgument('bag_name', default_value='lifelong_test',
                              description='bag 이름 접두어 (타임스탬프 자동 부착)'),
        DeclareLaunchArgument('launch_odometry', default_value='false',
                              description='true: VO(rgbd_odometry) 기동·기록. false(기본): 외부 EKF /odometry/filtered 기록'),
        DeclareLaunchArgument('monitor', default_value='false',   # rqt_image_view 는 헤드리스/배포에 없음
                              description='녹화 중 컬러 영상 미리보기 창 (rqt_image_view)'),
        OpaqueFunction(function=_setup),
    ])
