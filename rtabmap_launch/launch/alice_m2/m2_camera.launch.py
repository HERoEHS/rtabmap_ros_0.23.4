#
# ALICE M2 카메라 세트 (매니저 camera_m2) — Orbbec Gemini N대 + 임시 장착 TF + 카메라 묶음.
#
# 설정은 config/alice_m2/m2_cameras.yaml 하나 (프레임·토픽·장치·장착 위치 — 파일 머리말 참고).
#   카메라마다: 드라이버 하나 (camera_name = name, namespace /<name>, 2초 시차 — /dev/shm 장치 락 경쟁 회피)
#   mount_tf.publish: parent → <name>_link 정적 TF (외부 TF 노드가 생기면 false)
#   2대 이상 + rgbd_images_topic: rgbd_sync×N → rgbdx_sync (한 컨테이너, intra-process) → rgbd_images_topic
# 연산은 카메라 칩에 넘긴다 (640x400 이라 가능): align_mode HW(칩 D2C) · HW 노이즈 필터 · color YUYV(MJPEG 디코드 없음)
#   · 포인트클라우드 끔. 드라이버가 내는 TF 는 <name>_link 아래 체인뿐 (publish_mount_tf:=false).
#
# 사용: ros2 launch rtabmap_launch m2_camera.launch.py [m2_config:=<yaml>]
#
import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import m2_config  # noqa: E402 — 같은 디렉토리 (nav2_distro 와 같은 방식)

# 드라이버 고정 인자 — M1 gemini_camera.launch.py 의 RGB-D 설정 + M2 카메라 칩 오프로드. yaml driver: 가 덮는다.
# 두 카메라가 같은 크기여야 한다 (rtabmap 멀티카메라 assert). QoS default = Reliable 발행 — 레포 안 구독자는 전부
# Best Effort 라 호환되고 RViz·rosbag(Reliable)도 붙는다.
_DRIVER = {
    'depth_registration': 'true', 'enable_frame_sync': 'true',
    'color_width': '640', 'color_height': '400', 'color_fps': '15', 'color_format': 'YUYV',
    'depth_width': '640', 'depth_height': '400', 'depth_fps': '15',
    'align_mode': 'HW',
    'enable_hardware_noise_removal_filter': 'true', 'enable_noise_removal_filter': 'false',
    'enable_spatial_filter': 'true', 'enable_temporal_filter': 'true',
    'enable_point_cloud': 'false',
    'color_qos': 'default', 'depth_qos': 'default',
    'color_camera_info_qos': 'default', 'depth_camera_info_qos': 'default',
    'enable_accel': 'false', 'enable_gyro': 'false', 'enable_sync_output_accel_gyro': 'false',
}


def _selector(dev):
    """IP 면 PoE(335Le), 아니면 USB serial(336L). device_num '첫 장치' 폴백은 쓰지 않는다 — 앞뒤가 바뀐다."""
    import ipaddress
    try:
        ipaddress.ip_address(dev)
    except ValueError:
        return {'serial_number': dev}
    return {'net_device_ip': dev, 'net_device_port': '8090', 'enumerate_net_device': 'true', 'device_access_mode': 'EA'}


def _setup(context, *_):
    cfg = m2_config.load(LaunchConfiguration('m2_config').perform(context))
    cams = cfg['cameras']
    acts = [LogInfo(msg=f"[m2_camera] {cfg['path']} — " + ', '.join(f"{c['name']}={c['device']}" for c in cams))]
    try:
        driver = os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')
    except PackageNotFoundError:
        raise RuntimeError('[m2_camera] orbbec_camera 패키지 없음 — OrbbecSDK_ROS2 를 빌드할 것')

    for i, c in enumerate(cams):
        args = {**_DRIVER, **cfg['driver'], 'camera_name': c['name'], 'publish_mount_tf': 'false', **_selector(c['device'])}
        acts.append(TimerAction(period=2.0 * i, actions=[GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver), launch_arguments=args.items())])]))

    if cfg['mount_tf']:
        acts.append(LogInfo(msg='[m2_camera] 임시 장착 TF 를 낸다 (mount_tf.publish: true) — '
                                '외부 TF 노드가 <name>_link 를 내기 시작하면 yaml 에서 false 로'))
        for c in cams:
            (x, y, z), (roll, pitch, yaw) = c['xyz'], c['rpy']
            acts.append(Node(
                package='tf2_ros', executable='static_transform_publisher', name=f"{c['name']}_mount_tf",
                arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                           '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                           '--frame-id', c['parent'], '--child-frame-id', f"{c['name']}_link"]))

    if len(cams) > 1 and cfg['rgbd_images_topic']:
        # rtabmap rgbd_cameras:=N 은 이 빌드에서 안 된다(RTABMAP_SYNC_MULTI_RGBD=OFF) — rgbd_sync×N → rgbdx_sync 로 묶는다.
        # qos 2 = Best Effort (rtabmap 기본 qos 와 짝). color/depth 는 한 프레임셋이라 카메라 안은 간격 상한 없이 짝짓는다.
        intra = [{'use_intra_process_comms': True}]
        common = {'qos': 2, 'topic_queue_size': 10, 'sync_queue_size': 10, 'approx_sync': True}
        syncs = [ComposableNode(
            package='rtabmap_sync', plugin='rtabmap_sync::RGBDSync', name=f'rgbd_sync{i}', namespace='m2_rgbd',
            parameters=[{**common, 'qos_camera_info': 2, 'approx_sync_max_interval': 0.0}],
            remappings=[('rgb/image', f"/{c['name']}/color/image_raw"), ('depth/image', f"/{c['name']}/depth/image_raw"),
                        ('rgb/camera_info', f"/{c['name']}/color/camera_info"),
                        ('rgbd_image', f'rgbd_image{i}'), ('rgbd_image/compressed', f'rgbd_image{i}/compressed')],
            extra_arguments=intra) for i, c in enumerate(cams)]
        rgbdx = ComposableNode(
            package='rtabmap_sync', plugin='rtabmap_sync::RGBDXSync', name='rgbdx_sync', namespace='m2_rgbd',
            parameters=[{**common, 'rgbd_cameras': len(cams), 'approx_sync_max_interval': cfg['sync_max_interval']}],
            remappings=[('rgbd_images', cfg['rgbd_images_topic'])],
            extra_arguments=intra)
        acts.append(ComposableNodeContainer(
            name='rgbd_sync_container', namespace='m2_rgbd', package='rclcpp_components',
            executable='component_container', composable_node_descriptions=syncs + [rgbdx], output='screen'))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('m2_config', default_value=m2_config.default_path(),
                              description='M2 카메라 설정 yaml (기본 = env AEIROBOT_M2_CAMERAS, 없으면 '
                                          'rtabmap_launch config/alice_m2/m2_cameras.yaml)'),
        OpaqueFunction(function=_setup),
    ])
