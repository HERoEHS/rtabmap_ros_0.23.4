#
# ALICE M2 RTAB-Map (매니저 lifelong_mapping_m2, 벤치 단독 실행) — M2 카메라 yaml(aeirobot_orbbec_camera config/alice_m2/cameras.yaml)로 aeirobot_rtabmap 을 띄운다.
#
#   카메라 2대 이상: aeirobot_rtabmap 이 카메라 묶음(rgbd_images_topic)을 받는다 (3D-3D 등록 — aeirobot_rtabmap 주석)
#   주 카메라 cameras[0]: VO 입력 (odom_topic '' 일 때), multi_camera:=false 면 rtabmap 도 이 한 대만 (A/B)
#   기준 프레임: base_frame → frame_id
#   외부 odom(로봇·시뮬): odom 은 TF 로 읽고 링크 분산 1e-4, 라이다 점군(m2_scan_merge)도 — m2_config 주석
# launch_camera:=true(기본) 면 m2_camera 도 같이 띄운다 — 벤치 단독. 매니저는 camera_m2 가 따로라 false.
# m2_camera·m2_config(설정 읽기)는 aeirobot_orbbec_camera share 의 launch/ 에 있다 (package.xml exec_depend).
# localization·database_path·force_3dof·rviz·rtabmap_viz 등 나머지 인자는 aeirobot_rtabmap 으로 그대로 간다.
#
# 사용: ros2 launch rtabmap_launch m2_rtabmap.launch.py localization:=false force_3dof:=false   (노트북 핸드헬드)
#
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_HERE = os.path.dirname(os.path.realpath(__file__))
_M2_LAUNCH = os.path.join(get_package_share_directory('aeirobot_orbbec_camera'), 'launch')
sys.path.insert(0, _M2_LAUNCH)
import m2_config  # noqa: E402


def _setup(context, *_):
    path = LaunchConfiguration('m2_config').perform(context)
    cfg = m2_config.load(path)
    multi = LaunchConfiguration('multi_camera').perform(context).lower() in ('true', '1')
    acts = []
    if LaunchConfiguration('launch_camera').perform(context).lower() in ('true', '1'):
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_M2_LAUNCH, 'm2_camera.launch.py')),
            launch_arguments={'m2_config': path}.items()))
    lidar = LaunchConfiguration('lidar_scan').perform(context).lower() in ('true', '1')
    args = m2_config.rtabmap_args(cfg, multi, LaunchConfiguration('odom_topic').perform(context), lidar)
    acts.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(_HERE), 'aeirobot_rtabmap.launch.py')),
        launch_arguments=args.items()))
    merge = m2_config.scan_merge_launch(args, context.launch_configurations.get('use_sim_time', 'false'),
                                        LaunchConfiguration('lidar_beam_time').perform(context))
    if merge:
        acts.append(merge)          # 라이다 점군 (외부 odom 일 때 — m2_config 주석)
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('m2_config', default_value=m2_config.default_path(),
                              description='M2 카메라 설정 yaml (기본 = env AEIROBOT_M2_CAMERAS, 없으면 '
                                          'aeirobot_orbbec_camera config/alice_m2/cameras.yaml)'),
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='m2_camera(드라이버·장착 TF·묶음)도 같이. 매니저는 camera_m2 가 따로라 false'),
        DeclareLaunchArgument('multi_camera', default_value='true',
                              description='false = cameras[0] 한 대만 rtabmap 에 (A/B 비교). 드라이버는 그대로'),
        DeclareLaunchArgument('odom_topic', default_value='',
                              description='odom 입력 — 비면 카메라 yaml 의 odom_topic (\'\' = 주 카메라 VO). 토픽을 주면 '
                                          'VO 를 끄고 그 odom (매니저 M2 프로파일: /odometry/filtered = odom_m2)'),
        DeclareLaunchArgument('lidar_scan', default_value='true',
                              description='외부 odom 이면 2D 라이다 2대 + dToF 2대 점군도 rtabmap 에 (m2_scan_merge). '
                                          'false = 카메라만 (종전). 카메라 VO(벤치)면 이 값과 상관없이 끈다'),
        DeclareLaunchArgument('lidar_beam_time', default_value='ydlidar', choices=['ydlidar', 'isaac', 'index'],
                              description='2D 라이다 빔 시각 규약 — ydlidar = 실물 드라이버(0° 에서 시작해 각도 증가), '
                                          'isaac = Isaac 평면 스캔(+10° 에서 시작해 각도 감소, 시뮬 프로파일), '
                                          'index = ROS 규약(배열 순서). m2_config LIDAR_BEAM_TIMES 주석'),
        OpaqueFunction(function=_setup),
    ])
