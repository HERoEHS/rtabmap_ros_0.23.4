# rtabmap 특징 추출 · 매칭 — 고르기와 조정

`aeirobot_rtabmap.launch.py` 가 세대 폴더(`alice_m1` · `alice_m2`)의 yaml 두 개를 읽는다.

| 파일 | 정하는 것 |
|---|---|
| `feature_matching.yaml` | 특징 추출 · 매칭 묶음 고르기(`active`), 묶음마다 다른 값: 특징점 수, 추출 · 매칭 임계, 루프 수락 문턱 |
| `rtabmap_params.yaml` | 그 밖의 rtabmap 코어 값 (격자, 메모리, 최적화, 감지율 …) |

- 고치는 곳: ASH 파라미터 탭 › SLAM (RTAB-Map). 배포본에서는 `~/.aeirobot/config/rtabmap/<세대>/` 가 같은 파일이다.
- 저장한 뒤 SLAM 프로세스를 다시 시작해야 적용된다.
- 우선순위 (뒤가 이긴다): `rtabmap_params.yaml` < `feature_matching.yaml` 의 고른 묶음 < launch 가 계산하는 값 < `rtabmap_args`.
- `feature_matching.yaml` 에 있는 키는 그 파일만 정한다. `rtabmap_params.yaml` 에 같은 키를 다른 값으로 적어 두면 launch 가 멈춘다.
- 무엇이 적용됐는지는 SLAM 시작 로그에 나온다. 묶음 줄은 launch 가 덮은 것(카메라 2대의 `Vis/CorNNType 5` 등)과 `rtabmap_args` 까지 반영한 최종값이다.

```
[aeirobot_rtabmap] 특징 묶음 aliked_lightglue (매핑 적용 값, …/feature_matching.yaml): Kp/DetectorStrategy=15 … Vis/MinInliers=50
[aeirobot_rtabmap] 참고: rtabmap_params.yaml 에 남은 옛 기본값은 안 쓰인다 (지워도 된다) — RGBD/OptimizeMaxError 3.0
[aeirobot_rtabmap] 주의: rtabmap_args 가 yaml 의 값을 덮는다 — RGBD/OptimizeMaxError 3.0 → 1.5
```

## 묶음 바꾸기

1. `feature_matching.yaml` 의 `active` 를 `sets` 의 이름으로 고친다 (`superpoint_superglue` ↔ `aliked_lightglue`).
2. 맵을 다시 만든다. 두 묶음은 디스크립터가 달라(256 ↔ 128차원) 맵을 서로 못 쓴다.
3. SLAM 프로세스를 다시 시작하고 시작 로그의 묶음 이름을 확인한다.

되돌리려면 `active` 를 되돌린다. 그 묶음으로 만든 맵은 그대로 다시 쓴다.

런치가 띄우지 않고 멈추는 경우:

| 메시지 | 뜻 | 할 일 |
|---|---|---|
| `feature_matching.yaml 가 없다` | 파일이 이 기기의 config 폴더에 없다 | 배포본은 `aeirobot up` 으로 다시 시딩, 개발 워크스페이스는 `rtabmap_launch` 를 다시 빌드 |
| `active: … 은 sets 에 없는 이름이다` | 이름 오타 | 메시지에 나온 이름 중 하나로 |
| `… 에 고쳐 둔 값이 있는데 이 키는 이제 …` | 옛 `rtabmap_params.yaml` 에 이 파일로 옮겨 간 키가 고친 값으로 남아 있다 | 쓸 값을 묶음에 적고 `rtabmap_params.yaml` 의 그 줄을 지운다. `Kp/DetectorStrategy` 면 `active` 로 고른다 |
| `Kp/DetectorStrategy 는 11(SuperPoint) 또는 15(ALIKED)여야 한다` | 묶음에 번호가 없거나 다른 번호다 | 복사해 온 묶음의 번호를 그대로 둔다 |
| `… 는 "Xxx/Yyy: 값" 한 줄이어야 함` · `아래는 "Xxx/Yyy: 값" 맵이어야 함` | 들여쓰기가 어긋났다 | 묶음 바로 아래는 `키: 값`, `mapping:` · `localization:` 아래도 `키: 값` |
| `맵 … 는 Kp/DetectorStrategy N 로 만들어졌는데` | 다른 묶음으로 만든 맵 | 맵을 다시 만들거나 `active` 를 되돌린다 |
| `SuperPoint 가중치가 없다` · `가중치 aliked-n16.pth 이 없다` · `ALIKED · LightGlue 스크립트가 없다` | 고른 묶음의 모델이 이 기기에 없다 | `feature_extractors/aliked_lightglue/README.md` |

## 조정 순서

같은 내용이 yaml 안에도 있다. `feature_matching.yaml` 맨 위에 증상별 순서와 로그 읽는 법이 있고, 값마다 그 줄 위에 올리면 · 내리면 어떻게 되는지가 적혀 있다.

한 번에 값 하나만 바꾼다. 원래 묶음을 고치지 말고 통째로 복사해 이름을 붙인 뒤 그 사본을 `active` 로 고른다.
`Kp/DetectorStrategy` 가 같으면 맵은 그대로 쓰므로 `active` 한 줄로 오가며 견줄 수 있다.

| 증상 | 먼저 볼 값 | 방향 |
|---|---|---|
| 맵이 휘거나 겹친다, 위치가 순간이동한다 (틀린 루프가 통과) | `RGBD/OptimizeMaxError` 다음 `Vis/MinInliers` | 오차비는 내리고 인라이어는 올린다 |
| 맵이 안 닫힌다, 위치를 잘 못 찾는다 (맞는 루프가 버려짐) | `Vis/MinInliers` 다음 `RGBD/OptimizeMaxError` | 인라이어는 내리고 오차비는 올린다. 그래도 안 되면 `Kp/MaxFeatures` · `Vis/MaxFeatures` 를 같이 올린다 |
| 특징 추출 · 매칭이 느리다, GPU 가 모자란다 | `rtabmap_params.yaml` 의 `Rtabmap/DetectionRate` | 내린다. 그 다음 `Kp/MaxFeatures` · `Vis/MaxFeatures` 를 같이 내린다 |

- 틀린 루프는 맵을 만들 때 더 해롭다 (맵에 남아 계속 끌어당긴다). 그래서 맵을 만들 때만 쓰는 값은 묶음의 `mapping:` 아래에 따로 둔다.
- 반복 무늬(같은 벽판이 이어진 곳)가 만든 틀린 루프 묶음은 이 문턱으로 못 거른다. 서로 맞아떨어져 오차비가 작게 나온다. 환경을 바꿔야 한다.

## 잰 값 (2026-11-17, Isaac 정답 자세와 비교, 위치를 찾은 순간의 오차 중앙값)

| 조건 | SuperPoint 묶음 | ALIKED 묶음 |
|---|---|---|
| 카메라 2대, 오차비 1.5 · 인라이어 30 (매핑 50) — M2 기본값 | 0.74 · 0.76 · 0.82 cm | 0.63 · 0.71 · 0.78 cm |
| 카메라 2대, 오차비 3.0 · 인라이어 30 | 0.74 · 0.80 · 0.83 cm | 0.86 · 1.38 · 1.52 cm |
| 카메라 1대, 오차비 3.0 · 인라이어 30 — M1 기본값 | 1.50 · 1.52 cm | 0.89 · 0.96 cm |

- ALIKED 묶음은 카메라 2대에서 오차비 3.0 이면 방향이 90° 쯤 다른 자리를 잇는 7 ~ 12 cm 틀린 루프(인라이어 30개 남짓)를 받아들인다. 오차비 1.5 만으로 걸러진다 (0.55 cm, 1회). 인라이어 50 만으로는 0.86 cm (1회).
- LightGlue 임계 `PyMatcher/Threshold` 0.1 → 0.3 은 버렸다: 카메라 1대 오차 0.9 → 2.1 ~ 2.4 cm, 벽면이 바뀐 장면에서 위치를 찾은 프레임 36 → 28 (2대) · 29 → 15 (1대), 실데이터 루프 32 → 27건.
- 같은 설정도 실행마다 0.1 ~ 0.2 cm 흔들린다 (RANSAC). 틀린 루프가 경계에 걸리면 0.7 cm 까지 벌어진다. 설정마다 3회는 돌려서 견준다.
- 한 방 · 한 경로의 시뮬 기록이다. 실기기와 다른 환경에서는 재지 않았다. 카메라 1대 값은 M2 기록에서 카메라 하나만 쓴 것이다.

## 하지 않는 것

- `Kp/DetectorStrategy` 를 고쳐서 묶음을 바꾸지 않는다. 묶음은 `active` 로 고른다 (스크립트 · 모델 경로가 그 번호에 묶여 있다).
- `RGBD/OptimizeMaxError: 0` 은 검사를 끈다. 쓰지 않는다.
- 루프 수락 값을 프로필의 `rtabmap_args` 로 넘기지 않는다. yaml 값을 덮어서 ASH 에서 고쳐도 안 먹는다 (M2 가 전에 그랬다).
  남은 예외는 M1 시뮬 위치추정 프로필 `lifelong_zed` 하나다 (시뮬만 오차비 1.5, 실물은 3.0). 시작 로그의 「주의」 줄에 나온다.
- ALIKED 추출기의 환경변수(`ALIKED_MAX_KEYPOINTS` · `ALIKED_DETECTION_THRESHOLD`)는 잰 적이 없다. 기본값으로 둔다.

## 시험하는 법

1. 시뮬 매핑 프로필(`lifelong_mapping_sim_m2`)은 정답 자세를 맵 노드마다 DB 에 남긴다 (`ground_truth_topic`). 맵 하나, 위치추정용 주행 하나를 기록한다.
2. 고친 묶음으로 같은 기록을 `rtabmap-reprocess` 로 다시 돌린다. 넣는 값은 `rtabmap_params.yaml` 의 `rtabmap:` 절에 시작 로그의 묶음 줄을 덧씌운 것이다. 로그 줄은 최종값이라 yaml 만 읽고 넣으면 실제와 다른 값으로 재게 된다 (카메라 2대의 SuperPoint 묶음은 `Vis/CorNNType` 이 5 다).
3. 맵의 최적화 자세를 정답에 맞춘 뒤, 위치를 찾은 프레임의 자세와 정답의 거리를 본다. 위치추정률(찾은 프레임 비율)과 같이 본다.
