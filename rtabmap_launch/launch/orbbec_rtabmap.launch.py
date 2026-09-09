#
# Orbbec Gemini 330 시리즈 (330L/336L) + RTAB-Map
#
# rtabmap_with_gui.launch.py 구조를 따르되 orbbec RGB-D 입력에 맞게 변경:
#   1. 스테레오 입력 → RGB-D 입력 (subscribe_depth)
#   2. 외부 odom(/odometry/filtered) → rtabmap_odom rgbd_odometry (VO) 노드 추가
#   3. 카메라 드라이버(gemini_330_series.launch.py)를 depth_registration:=true 로 포함 실행
#   4. localization argument로 매핑/위치추정 모드 선택 (GUI 서비스 전환 대신 런치 인자)
#   5. force_3dof argument: 로봇 탑재 시 true, 손으로 들고 테스트 시 false
#   6. camera argument: orbbec | zed — 카메라 종류는 sim/실물과 **별개**의 축이다.
#      프리셋은 토픽 3종 + 드라이버 기동 여부만. 기본값은 env AEIROBOT_CAMERA
#      (배포본 ~/.aeirobot/drive.env), 없으면 zed.
#   7. odom 소스는 카메라와 무관하게 기본 외부 EKF(/odometry/filtered). VO 는 launch_odometry:=true.
#
# 사용법:
#   매핑:     ros2 launch rtabmap_launch orbbec_rtabmap.launch.py
#   위치추정: ros2 launch rtabmap_launch orbbec_rtabmap.launch.py localization:=true
#   Orbbec:   ros2 launch rtabmap_launch orbbec_rtabmap.launch.py camera:=orbbec
#   (기본 zed. sim 이면 use_sim_time:=true 추가)
#
# 토픽 (camera_name=camera 기준):
#   rgb   : /camera/color/image_raw
#   depth : /camera/depth/image_raw   (depth_registration:=true → color에 정렬됨)
#   info  : /camera/color/camera_info
#

import os

# ROS_WS 환경변수 필수
ROS_WS = os.environ.get('ROS_WS')
if not ROS_WS:
    raise RuntimeError("ROS_WS 환경변수가 설정되지 않았습니다. 예: export ROS_WS=/path/to/ws")
FEATURE_EXTRACTORS = os.path.join(ROS_WS, 'src', 'alice_navigation', 'localization', 'feature_extractors')

# 맵 DB 저장 위치 (slam_manager 워크플로와 동일한 맵 루트 slam/).
# ~/.ros 를 안 쓰는 이유는 aeirobot_slam_manager/lifelong_maps.py 헤더 주석 참고.
MAP_ROOT = os.environ.get('AEIROBOT_MAP_ROOT') or os.path.expanduser('~/.aeirobot/maps')
MAP_DIR = os.path.join(MAP_ROOT, 'slam')
os.makedirs(MAP_DIR, exist_ok=True)

import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration, TimerAction)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# ── 카메라 프리셋 (camera:=orbbec|zed) — sim/실물 축과도, odom 소스 축과도 독립 ──
# 프리셋이 정하는 건 **토픽 3종 + 드라이버를 여기서 띄우는지** 뿐이다.
#   orbbec: 실물 Gemini, /camera/* — 드라이버(gemini_330_series)를 이 launch 가 띄운다
#   zed:    실물 zed2i / Isaac ZED, /aeirobot/vslam_* — 드라이버는 외부(aeirobot_zed_camera / Isaac)
# odom 소스는 카메라와 무관하게 **기본 외부 EKF(/odometry/filtered)** 다. 로봇에 달린 카메라는
# 둘 다 EKF 를 쓴다 — orbbec 이 VO 를 썼던 건 로봇 미탑재 손테스트 시절 잔재. VO 가 필요하면
# launch_odometry:=true 만 주면 odom_topic(odom)·subscribe_odom_info(true)·use_imu(orbbec 만 true)
# 가 따라온다. 같은 이름 인자를 명시하면(빈 값이 아니면) 항상 인자가 이긴다.
_CAM_PRESETS = {
    'orbbec': {
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/depth/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        'launch_camera': 'true',
    },
    'zed': {
        'rgb_topic': '/aeirobot/vslam_left_image',
        'depth_topic': '/aeirobot/vslam_depth',
        'camera_info_topic': '/aeirobot/vslam_left_camera_info',
        'launch_camera': 'false',
    },
}
_CAMERA_DEFAULT = os.environ.get('AEIROBOT_CAMERA', 'zed')


def _apply_camera_preset(context, *_):
    """빈 인자만 채운다 (명시 인자 우선): 카메라 프리셋 → odom 소스 기본값(launch_odometry 기준)."""
    cam = LaunchConfiguration('camera').perform(context)
    if cam not in _CAM_PRESETS:
        raise RuntimeError(f"[orbbec_rtabmap] camera:={cam!r} — 가능한 값: {', '.join(_CAM_PRESETS)}")
    resolved = dict(_CAM_PRESETS[cam])
    vo = (LaunchConfiguration('launch_odometry').perform(context) or 'false').lower() in ('true', '1')
    if vo:
        # rgbd_odometry 는 odom→base TF 를 발행한다(publish_tf 기본 true). EKF(robot_localization,
        # publish_tf true)와 같이 돌면 같은 에지를 둘이 쓰고, rtabmap 의 map→odom 은 VO odom 기준이라
        # map→base 가 틀어진다. VO 는 외부 odom 이 없는 구성(핸드헬드·bag) 전용 — 매니저 프로파일은
        # ekf 프로세스를 항상 띄우므로 launch_odometry:=true 를 넣지 말 것.
        print('[orbbec_rtabmap] VO 모드(launch_odometry:=true): rgbd_odometry 가 odom→base TF 를 발행한다. '
              'EKF(wio_ekf) 와 동시 기동 금지 — 핸드헬드/bag 전용.')
    resolved.update({
        'launch_odometry': 'true' if vo else 'false',
        'odom_topic': 'odom' if vo else '/odometry/filtered',
        'subscribe_odom_info': 'true' if vo else 'false',   # 외부 odom 은 odom_info 미발행
        'use_imu': 'true' if (vo and cam == 'orbbec') else 'false',   # madgwick 은 VO 중력 정렬용
    })
    return [SetLaunchConfiguration(k, v) for k, v in resolved.items()
            if LaunchConfiguration(k).perform(context) == '']


def _load_tuning(path):
    """config/rtabmap_params.yaml → {노드이름: {'Xxx/Yyy': '문자열'}}.

    rtabmap 코어 파라미터는 전부 string 선언이라 yaml 타입추론(0.11→double, false→bool)이
    그대로 노드에 닿으면 declare 시 타입 충돌로 죽는다. 여기서 문자열로 정규화해 그 함정을
    없앤다 — 고객이 따옴표를 빼도 안전. 배포본은 launch 본문이 .so 로 봉인되므로 튠 값은
    이 yaml 이 유일한 손잡이다 (docker/docs/55-user-config.md)."""
    def _s(v):
        if isinstance(v, bool):
            return 'true' if v else 'false'
        return str(v)
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'[orbbec_rtabmap] {path}: 최상위는 노드별 맵(rtabmap:/rgbd_odometry:)이어야 함')
    out = {}
    for node, params in data.items():
        if params is None:
            params = {}
        if not isinstance(params, dict):
            raise RuntimeError(f'[orbbec_rtabmap] {path}: {node}: 아래는 "Xxx/Yyy: 값" 맵이어야 함')
        out[node] = {k: _s(v) for k, v in params.items()}
    return out


def launch_setup(context, *args, **kwargs):

    localization = LaunchConfiguration('localization')
    force_3dof = LaunchConfiguration('force_3dof')

    wait_imu = (LaunchConfiguration('wait_imu_to_init').perform(context)
                or LaunchConfiguration('use_imu').perform(context)).lower() in ('true', '1')

    tuning = _load_tuning(LaunchConfiguration('rtabmap_params').perform(context))
    # detection_rate:=N 이 명시되면 yaml 의 Rtabmap/DetectionRate 를 덮는다 (빈 값 = yaml).
    detection_rate = LaunchConfiguration('detection_rate').perform(context)
    detection_override = {'Rtabmap/DetectionRate': detection_rate} if detection_rate else {}

    # 카메라가 sensor_data(Best Effort) QoS로 발행하므로 맞춰줌
    # rgbd_odometry는 이미지 QoS를 'qos'로, rtabmap/viz는 'qos_image'로 받음
    common_params = {
        'frame_id': LaunchConfiguration('frame_id'),
        'subscribe_depth': True,
        # bag 재생 검증 시 false (bag에 odom_info 미수록 — true면 동기화 무한 대기)
        'subscribe_odom_info': LaunchConfiguration('subscribe_odom_info'),
        'approx_sync': LaunchConfiguration('approx_sync'),
        'wait_for_transform': LaunchConfiguration('wait_for_transform'),
        'topic_queue_size': LaunchConfiguration('topic_queue_size'),
        'sync_queue_size': LaunchConfiguration('sync_queue_size'),
        # 기본 2(Best Effort) = 라이브 카메라용. bag 재생은 qos:=1(Reliable) 로 —
        # 고해상도 raw 버스트 + 시스템 부하 시 best-effort 가 depth 만 드랍해
        # sync 가 조용히 죽는다 (실측: best-effort 3건 vs reliable 341/341).
        'qos': ParameterValue(LaunchConfiguration('qos'), value_type=int),
        'qos_image': ParameterValue(LaunchConfiguration('qos'), value_type=int),
        'qos_camera_info': ParameterValue(LaunchConfiguration('qos'), value_type=int),
        'qos_odom': ParameterValue(LaunchConfiguration('qos'), value_type=int),
    }

    remappings = [
        ('rgb/image', LaunchConfiguration('rgb_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_topic')),
        ('imu', LaunchConfiguration('imu_topic')),
    ]

    return [
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        ### IMU orientation 필터 (orbbec IMU는 raw만 발행 — rtabmap은 orientation 필요) ###
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter', output='screen',
            parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/gyro_accel/sample'),
                        ('imu/data', LaunchConfiguration('imu_topic'))],
            condition=IfCondition(LaunchConfiguration('use_imu'))),

        ### Visual Odometry (rtabmap 제공 RGB-D VO — 기본 GFTT, SuperPoint는 rtabmap 노드 전용) ###
        Node(
            condition=IfCondition(LaunchConfiguration('launch_odometry')),
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
            # 튠 값(Odom/ResetCountdown 등)은 config/rtabmap_params.yaml rgbd_odometry: 절.
            parameters=[dict(common_params, **tuning.get('rgbd_odometry', {}), **{
                'wait_imu_to_init': wait_imu,
                'Reg/Force3DoF': ParameterValue(force_3dof, value_type=str),
            })],
            remappings=remappings,
            # 로거는 네임스페이스 포함 이름(rtabmap.rgbd_odometry) — 노드명만 쓰면 무효과 (2026-07-23 실측)
            arguments=["--ros-args", "--log-level",
                       [LaunchConfiguration('namespace'), '.rgbd_odometry:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Visual SLAM (with_gui와 동일하게 3초 지연 시작) ###
        TimerAction(period=3.0, actions=[
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            # 튠 값(Grid/Vis/Kp/SuperPoint/Optimizer/LC 임계 …)은 config/rtabmap_params.yaml rtabmap: 절.
            # 여기 남은 것은 launch 인자·환경으로 계산되는 값만 — yaml 보다 뒤라 yaml 을 덮는다.
            parameters=[dict(common_params, **tuning.get('rtabmap', {}), **{
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'publish_tf': LaunchConfiguration('publish_tf_map'),
                'initial_pose': LaunchConfiguration('initial_pose'),
                'database_path': LaunchConfiguration('database_path'),
                'odom_sensor_sync': LaunchConfiguration('odom_sensor_sync'),
                'config_path': LaunchConfiguration('cfg').perform(context),
                'odom_correction': LaunchConfiguration('odom_correction'),

                # ============================================================
                # 모드 선택: localization argument (with_gui는 GUI 서비스로 전환)
                # ParameterValue(str) 필수: substitution 결과는 yaml 타입추론으로
                # bool이 되어 rtabmap의 string 선언과 충돌
                # ============================================================
                'Mem/IncrementalMemory': ParameterValue(
                    PythonExpression(["'false' if '", localization, "' == 'true' else 'true'"]), value_type=str),
                # WM 노드 수 상한(0=무제한). VO 발산 등으로 전역을 가상 이동해도
                # WM 캐시가 맵 전체로 퍼지지 못하게 하는 안전벨트 (setup 3.29)
                'Rtabmap/MemoryThr': ParameterValue(LaunchConfiguration('memory_thr'), value_type=str),
                'Mem/InitWMWithAllNodes': ParameterValue(
                    PythonExpression(["'true' if '", localization, "' == 'true' else 'false'"]), value_type=str),

                # 3DoF 강제는 평면 주행 로봇 전용. 손으로 들고 테스트하면 pitch/roll/z가
                # 깎여서 이동량이 노드 생성 임계값에 못 미쳐 맵이 안 자람 (WM=1 고정 증상)
                'Reg/Force3DoF': ParameterValue(force_3dof, value_type=str),
                'RGBD/ForceOdom3DoF': ParameterValue(force_3dof, value_type=str),

                # 모델 경로는 워크스페이스 위치(ROS_WS)에 묶여 있어 launch 가 계산한다
                'SuperPoint/ModelPath': os.path.join(FEATURE_EXTRACTORS, 'superpoint_v1.pt'),
                'PyMatcher/Path': os.path.join(FEATURE_EXTRACTORS, 'SuperGluePretrainedNetwork', 'rtabmap_superglue.py'),
            }, **detection_override)],
            # odom_topic 기본 'odom'(상대) = 종전 그대로 VO(/rtabmap/odom) 소비.
            # 외부 odom(EKF 등) 쓸 땐 launch_odometry:=false odom_topic:=/odometry/filtered
            remappings=remappings + [('map', LaunchConfiguration('map_topic')),
                                     ('odom', LaunchConfiguration('odom_topic'))],
            # 매핑 모드는 -d(기존 DB 삭제 후 새로 시작), localization 모드는 DB 유지·로드
            arguments=[
                LaunchConfiguration('rtabmap_args'),
                PythonExpression(["'' if '", localization, "' == 'true' else '-d'"]),
                "--ros-args", "--log-level",
                [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace'),
            # HERoEHS lifelong: 스레드 풀 상한 (CPU 예산 — setup 3.44).
            # OpenCV/PCL/BLAS가 제한 없이 코어 수만큼 풀을 열어 스레드 433개 중
            # 36개가 각 3~9%씩 소모 중이었다(실측 206%). 상위 런치의
            # SetEnvironmentVariable은 이 노드가 TimerAction 안에 있어 스코프가
            # 닿지 않으므로 노드에 직접 주입한다.
            # 값은 코어 수가 아니라 **CPU 예산**에서 유도 — Orin/Thor 이식 가능.
            additional_env={
                'OMP_NUM_THREADS': LaunchConfiguration('rtabmap_threads'),
                'OPENBLAS_NUM_THREADS': LaunchConfiguration('rtabmap_threads'),
                'MKL_NUM_THREADS': LaunchConfiguration('rtabmap_threads'),
                # HERoEHS lifelong: glibc 아레나 상한 (메모리 성장 대책 — setup 3.78).
                # rtabmap은 스레드 433개(3.44 실측)라 아레나가 코어 수 기준으로 열려
                # 할당·해제 churn이 OS로 반환되지 않고 RSS로 쌓인다. khronos는 3.30에서
                # 같은 처방(MALLOC_ARENA_MAX=4)으로 GB급 성장을 잡았는데 rtabmap엔
                # 미적용 상태였다. glibc는 0을 범위 밖으로 무시하므로 0 = 미적용(A/B용).
                'MALLOC_ARENA_MAX': LaunchConfiguration('rtabmap_arena_max'),
            }),
        ]),  # TimerAction 닫기

        ### Rtabmap GUI ###
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
            parameters=[common_params],
            # odom remap 은 rtabmap 노드와 동일하게 — 외부 odom(EKF) 구성에서
            # 기본 /rtabmap/odom 만 기다리면 동기화가 영영 안 찬다 (5초 경고 반복)
            remappings=remappings + [('odom', LaunchConfiguration('odom_topic'))],
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            arguments=[LaunchConfiguration('gui_cfg')],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Rviz ###
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            # sim clock 이면 rviz 도 sim time 이어야 TF 가 맞는다 (없으면 벽시계로 조회해 전부 too old)
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]),
    ]


def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    return LaunchDescription([

        # Arguments
        ## 모드 선택
        DeclareLaunchArgument('localization', default_value='true', description='true: 기존 DB로 위치추정 모드, false: 매핑 모드'),
        DeclareLaunchArgument('force_3dof',   default_value='false', description='true: 평면(3DoF) 강제 — 로봇 탑재 시 사용. 손으로 들고 테스트할 땐 false'),
        DeclareLaunchArgument('rtabmap_params',
                              default_value=os.path.join(get_package_share_directory('rtabmap_launch'),
                                                         'config', 'rtabmap_params.yaml'),
                              description='rtabmap/rgbd_odometry 코어 파라미터 튠 yaml (노드별 평면 맵, 값은 문자열로 정규화)'),
        DeclareLaunchArgument('detection_rate', default_value='',
                              description='loop closure 감지율 [Hz]. 빈 값(기본)=yaml 의 Rtabmap/DetectionRate. '
                                          'GPU 경합 시(라이프롱 스택 노트북 구동) 1 권장'),
        DeclareLaunchArgument('memory_thr', default_value='0',
                              description='Rtabmap/MemoryThr — WM 노드 수 상한(0=무제한). 장시간 localization 운영 시 350 권장 (setup 3.29)'),
        DeclareLaunchArgument('rtabmap_threads', default_value='0',
                              description='rtabmap의 OpenCV/PCL/BLAS 스레드 풀 상한. '
                                          '0=제한 없음(코어 수만큼 = 상류 기본). CPU 예산이 있는 '
                                          '배포에선 4 권장 — 코어 수가 아니라 예산에서 유도한 값이라 '
                                          'Orin/Thor 이식 가능 (setup 3.44)'),
        DeclareLaunchArgument('rtabmap_arena_max', default_value='0',
                              description='rtabmap 노드의 glibc MALLOC_ARENA_MAX. '
                                          '0=미적용(glibc가 범위 밖으로 무시 = 상류 기본). '
                                          '메모리 성장 대책은 4 — khronos 검증 처방과 동일 (setup 3.78)'),

        ## GUI ON / OFF
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',        default_value='true', description='Launch RVIZ (optional).'),

        ## odom tf 보정
        DeclareLaunchArgument('odom_correction', default_value='true', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('launch_odometry', default_value='',   # 빈 값 = false (외부 EKF odom)
                              description='rgbd_odometry 실행 여부. bag 재생 검증(odom이 bag에 있음)이면 false'),
        DeclareLaunchArgument('subscribe_odom_info', default_value='',
                              description='odom_info 구독. bag 재생 검증(미수록)이면 false'),
        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error)."),

        # Config files
        DeclareLaunchArgument('cfg',      default_value='',                        description='RTAB-Map config file path (*.ini).'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='base_footprint',    description='기준 프레임 (카메라 런치가 base_footprint->camera_link tf 발행)'),
        DeclareLaunchArgument('map_frame_id',   default_value='map',               description=''),
        DeclareLaunchArgument('map_topic',      default_value='map',               description=''),
        DeclareLaunchArgument('publish_tf_map', default_value='true',              description=''),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',           description=''),
        DeclareLaunchArgument('database_path',  default_value=os.path.join(MAP_DIR, 'orbbec_rtabmap.db'),
                              description='맵 DB 경로 (매핑 모드는 시작 시 삭제 후 새로 생성, localization 모드는 로드). 예: database_path:=~/.aeirobot/maps/slam/field_x.db'),
        DeclareLaunchArgument('qos', default_value='2', description='구독 QoS: 2=BestEffort(라이브 카메라), 1=Reliable(bag 재생 — 드랍 방지)'),
        DeclareLaunchArgument('topic_queue_size', default_value='10',              description=''),
        DeclareLaunchArgument('sync_queue_size',  default_value='10',              description=''),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',           description=''),
        DeclareLaunchArgument('rtabmap_args',   default_value='',                  description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug'),
        DeclareLaunchArgument('launch_prefix',  default_value='',                  description=''),
        DeclareLaunchArgument('initial_pose',   default_value='',                  description=''),
        DeclareLaunchArgument('odom_sensor_sync', default_value='false',           description=''),

        DeclareLaunchArgument('approx_sync', default_value='true', description='rgb/depth 근사 시간동기화 (enable_frame_sync 완벽하면 false 가능)'),

        # 외부 오도매트리 (sim/EKF): 기본 'odom'(상대 = VO 출력 /rtabmap/odom, 종전 동작).
        # launch_odometry:=false 와 함께 /odometry/filtered 등 절대 토픽 지정
        DeclareLaunchArgument('odom_topic', default_value='',
                              description='rtabmap odom 입력. 기본은 VO(rgbd_odometry) 출력. '
                                          '외부 odom(robot_localization EKF)이면 launch_odometry:=false 와 함께 지정'),

        # RGB-D 토픽 — 빈 값(기본)이면 camera 프리셋 (orbbec: /camera/*, zed: /aeirobot/vslam_*)
        DeclareLaunchArgument('rgb_topic',         default_value='', description='빈 값 = camera 프리셋'),
        DeclareLaunchArgument('depth_topic',       default_value='', description='빈 값 = camera 프리셋'),
        DeclareLaunchArgument('camera_info_topic', default_value='', description='빈 값 = camera 프리셋'),

        # imu (use_imu:=true → 카메라 내장 IMU 활성화 + madgwick 필터로 orientation 추정 → VO 중력 정렬)
        DeclareLaunchArgument('use_imu',          default_value='',             description='카메라 내장 IMU를 VO 중력 정렬에 사용 (imu_filter_madgwick 패키지 필요). 빈 값 = orbbec+VO 일 때만 true'),
        DeclareLaunchArgument('imu_topic',        default_value='/rtabmap/imu',  description='orientation이 채워진 IMU 토픽 (madgwick 필터 출력)'),
        # 기본 = use_imu. 빈 값이면 프리셋 적용 뒤 launch_setup 에서 use_imu 값을 따른다
        # (DeclareLaunchArgument 의 default 는 선언 시점에 굳어 프리셋 전 값 '' 을 잡는다).
        DeclareLaunchArgument('wait_imu_to_init', default_value='', description='빈 값 = use_imu 와 동일'),

        # 카메라 드라이버
        DeclareLaunchArgument('launch_camera', default_value='', description='카메라 드라이버 포함 실행 여부 (이미 켜져 있으면 false). 빈 값 = camera 프리셋'),

        # 카메라 종류 — sim/실물과 별개 축. 빈 인자를 프리셋·odom 기본값으로 여기서 채운다.
        # 이 OpaqueFunction 이 아래 노드/include 보다 먼저 와야 한다 (IfCondition·remap 이 그 값을 읽는다).
        DeclareLaunchArgument('camera', default_value=_CAMERA_DEFAULT,
                              description='orbbec | zed (토픽·드라이버만 바뀜, odom 소스는 별개). 기본 = env AEIROBOT_CAMERA, 없으면 zed'),
        OpaqueFunction(function=_apply_camera_preset),
    ] + _orbbec_camera_include() + [
        OpaqueFunction(function=launch_setup)
    ])


def _orbbec_camera_include():
    """Gemini 드라이버 — 인자 한 벌은 gemini_camera.launch.py (매니저 camera 프로세스와 공유).
    launch_camera 는 프리셋이 채운다(orbbec=true, zed=false). 매니저 아래선 false 로 넘어온다."""
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rtabmap_launch'), 'launch', 'gemini_camera.launch.py')),
            launch_arguments={'use_imu': LaunchConfiguration('use_imu')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),
    ]
