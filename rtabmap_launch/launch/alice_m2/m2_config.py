"""ALICE M2 카메라 설정(config/alice_m2/m2_cameras.yaml) 읽기.

alice_m2/m2_camera·m2_rtabmap (같은 디렉토리)와 aeirobot_lifelong alice_m2/m2_lifelong 이 같이 쓴다.
빠진 키는 기본값 — 고친 호스트 파일엔 새 키가 안 들어가므로 cameras 말고는 전부 선택이다.
"""
import os

import yaml


def default_path():
    """env AEIROBOT_M2_CAMERAS (노트북 벤치 serial 사본 등) 가 있으면 그것, 없으면 rtabmap_launch 의 기본 파일."""
    from ament_index_python.packages import get_package_share_directory
    return os.environ.get('AEIROBOT_M2_CAMERAS') or os.path.join(
        get_package_share_directory('rtabmap_launch'), 'config', 'alice_m2', 'm2_cameras.yaml')


def _s(v):
    """launch 인자는 문자열 — yaml 이 bool/숫자로 읽은 값을 되돌린다."""
    return ('true' if v else 'false') if isinstance(v, bool) else str(v)


def load(path):
    path = os.path.expanduser(path)
    with open(path, encoding='utf-8') as f:
        cfg = (yaml.safe_load(f) or {}).get('m2_cameras') or {}
    cams = cfg.get('cameras') or []
    names = [c.get('name') for c in cams]
    if not cams:
        raise RuntimeError(f'[m2] {path}: cameras 가 비었다')
    if not all(names) or len(set(names)) != len(names):
        raise RuntimeError(f'[m2] {path}: 카메라 name 이 비었거나 겹친다: {names}')
    for c in cams:
        if not str(c.get('device') or '').strip():
            # device 없이 열면 드라이버가 '첫 장치' 를 잡아 카메라 앞뒤가 바뀐다
            raise RuntimeError(f"[m2] {path}: {c['name']} 의 device(IP 또는 serial)가 없다")
    base = cfg.get('base_frame') or 'base_footprint'
    rgbd = cfg.get('rgbd_images_topic')
    # 따옴표 친 "false" 를 참으로 읽으면 외부 TF 노드와 장착 TF 가 겹쳐 camera_0N_link 부모가 둘이 된다
    pub = (cfg.get('mount_tf') or {}).get('publish', True)
    if not isinstance(pub, bool):
        raise RuntimeError(f"[m2] {path}: mount_tf.publish 는 따옴표 없는 true/false 여야 한다 (지금 {pub!r})")
    return {
        'path': path,
        'base_frame': base,
        'odom_topic': cfg.get('odom_topic') or '',
        # 절대 이름으로 — 묶음(ns m2_rgbd)과 rtabmap(ns rtabmap)이 상대 이름이면 서로 다른 토픽이 된다
        'rgbd_images_topic': '/m2/rgbd_images' if rgbd is None else ('/' + rgbd.lstrip('/') if rgbd else ''),
        'sync_max_interval': float(cfg.get('sync_max_interval', 0.05)),
        'mount_tf': pub,
        'driver': {k: _s(v) for k, v in (cfg.get('driver') or {}).items()},
        'cameras': [{'name': c['name'], 'device': str(c['device']).strip(), 'parent': c.get('parent') or base,
                     'xyz': [float(v) for v in (c.get('xyz') or [0.0, 0.0, 0.0])],
                     'rpy': [float(v) for v in (c.get('rpy') or [0.0, 0.0, 0.0])]} for c in cams],
    }


def rtabmap_args(cfg, multi_camera=True):
    """aeirobot_rtabmap / aeirobot_lifelong 에 넘길 카메라·프레임·odom 인자 (cameras[0] = 주 카메라)."""
    c0 = cfg['cameras'][0]['name']
    multi = multi_camera and len(cfg['cameras']) > 1 and bool(cfg['rgbd_images_topic'])
    odom = cfg['odom_topic']
    return {
        'camera': 'orbbec_m2', 'launch_camera': 'false', 'use_imu': 'false',   # 카메라 IMU·madgwick 안 씀
        'rgb_topic': f'/{c0}/color/image_raw',
        'depth_topic': f'/{c0}/depth/image_raw',
        'camera_info_topic': f'/{c0}/color/camera_info',
        'frame_id': cfg['base_frame'],
        'rgbd_images_topic': cfg['rgbd_images_topic'] if multi else '',
        # odom: '' 면 cameras[0] VO, 토픽이 있으면 VO 끄고 그 odom
        'launch_odometry': 'false' if odom else 'true',
        **({'odom_topic': odom} if odom else {}),
    }
