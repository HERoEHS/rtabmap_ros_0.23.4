#
# Orbbec Gemini 33x 드라이버 — lifelong/rtabmap 스택이 쓰는 고정 인자 한 벌.
#
# 호출처 두 곳이 같은 드라이버 설정을 써야 한다:
#   · orbbec_rtabmap.launch.py  (camera:=orbbec launch_camera:=true — 단독 CLI 기동)
#   · aeirobot_slam_manager camera.launch.py (매니저 camera 프로세스, AEIROBOT_CAMERA=orbbec)
# 인자를 두 군데 복사하면 해상도·QoS 가 갈라진다 — 여기서만 정한다.
#
# use_imu: 카메라 IMU 스트림(accel/gyro) 켜기 — VO 중력 정렬(madgwick)용. EKF odom 구성에선 불필요.
#
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(context, *_):
    """orbbec_camera 미빌드 머신(sim, zed)에서 파싱만으로 죽지 않게 가드."""
    try:
        launch_dir = os.path.join(get_package_share_directory('orbbec_camera'), 'launch')
    except PackageNotFoundError:
        print('[gemini_camera] orbbec_camera 패키지 없음 — 드라이버 생략 '
              '(sim/zed 구성이면 정상, 실물 Gemini 면 OrbbecSDK_ROS2 빌드 필요)')
        return []
    use_imu = LaunchConfiguration('use_imu')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/gemini_330_series.launch.py']),
        launch_arguments={'depth_registration': 'true',
                          'enable_frame_sync': 'true',
                          # 640x480@15 고정: 네이티브(1280x800)는 픽셀 3.3배라 VO 단일스레드가
                          # 코어 포화로 3.9Hz까지 붕괴 (2026-07-22 풀스택 실측). 해상도가 최대 지렛대
                          'color_width': '640', 'color_height': '480', 'color_fps': '15',
                          'depth_width': '640', 'depth_height': '480', 'depth_fps': '15',
                          # depth 노이즈 필터 (실기 TSDF 메시 품질 — 시뮬 대비 괴리 완화)
                          'enable_spatial_filter': 'true',
                          'enable_temporal_filter': 'true',
                          # Reliable 발행: rtabmap(Best Effort 구독)과 Khronos(Reliable 구독) 모두 호환.
                          # sensor_data(Best Effort) 기본값이면 Khronos가 camera_info를 못 받아 초기화에 갇힘
                          'color_qos': 'default',
                          'depth_qos': 'default',
                          'color_camera_info_qos': 'default',
                          'depth_camera_info_qos': 'default',
                          'enable_accel': use_imu,
                          'enable_gyro': use_imu,
                          'enable_sync_output_accel_gyro': use_imu}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='카메라 IMU(accel/gyro) 스트림 — VO 중력 정렬용. EKF odom 이면 false'),
        OpaqueFunction(function=_include),
    ])
