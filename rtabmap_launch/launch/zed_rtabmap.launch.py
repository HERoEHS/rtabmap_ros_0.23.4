#
# ZED + rtabmap 부트스트랩 매핑 — orbbec_rtabmap.launch.py camera:=zed 의 얇은 별칭 (use_sim_time 기본 true)
#
# 실물 부트스트랩(orbbec_rtabmap.launch.py 직접 실행)의 sim 대응물:
#   카메라 드라이버/VO/카메라 IMU 없이, Isaac(sim.launch.py)이 발행하는
#   /aeirobot/vslam_left_* 토픽과 EKF odom(/odometry/filtered)을 소비한다.
#
# 사용법 (slam_manager lifelong_zed_mapping 프로세스가 이 형태로 실행):
#   ros2 launch rtabmap_launch zed_rtabmap.launch.py localization:=false database_path:=~/.aeirobot/maps/slam/xxx.db
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # rviz:=true 면 rtabmap 제공 rviz2(rgbd.rviz)를 rtabmap_viz 와 같이 띄운다.
        # 매니저 매핑 프로파일(lifelong_zed_mapping)만 켠다 — 로컬모드는 nav2 rviz 가 있다.
        DeclareLaunchArgument('rviz', default_value='false'),
        # 기본 true = Isaac sim (/clock). 실물 zed2i(Orin) 는 use_sim_time:=false — 그 외 배선 동일.
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='sim=true / 실물 zed2i=false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch', 'orbbec_rtabmap.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': LaunchConfiguration('rviz'),
                # 토픽·드라이버 없음·EKF odom·IMU 없음 배선은 camera=zed 프리셋이 채운다 (orbbec_rtabmap).
                'camera': 'zed',
                'force_3dof': 'true',             # 평면 주행 로봇 탑재 형상
            }.items()),
    ])
