#
# ALICE M2 RTAB-Map (매니저 lifelong_mapping_m2, 벤치 단독 실행) — M2 카메라 yaml(aeirobot_orbbec_camera config/alice_m2/cameras.yaml)로 aeirobot_rtabmap 을 띄운다.
#
#   카메라 2대 이상: aeirobot_rtabmap 이 카메라 묶음(rgbd_images_topic)을 받는다 (3D-3D 등록 — aeirobot_rtabmap 주석)
#   주 카메라 cameras[0]: VO 입력 (odom_topic '' 일 때), multi_camera:=false 면 rtabmap 도 이 한 대만 (A/B)
#   기준 프레임: base_frame → frame_id
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
    acts.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(_HERE), 'aeirobot_rtabmap.launch.py')),
        launch_arguments=m2_config.rtabmap_args(cfg, multi).items()))
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
        OpaqueFunction(function=_setup),
    ])
