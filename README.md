# grasping_on_time

MuJoCo와 Franka Panda를 이용한 컨베이어 물체 파지 실험 환경입니다. 직선, 원형, 모서리가 둥근 정사각형 컨베이어를 JSON 설정으로 선택합니다.

현재 기본 실험은 **벨트 이동 → 정지 → 상자 위치 측정 → 접근 → 파지 → 들어 올리기**입니다. 이동 중인 물체를 예측해서 잡는 기능은 아직 포함하지 않습니다. 컨베이어는 접촉 중인 상자에 힘을 적용하는 근사 모델이며, 현재 위치의 벨트 진행 방향을 따라가도록 제어합니다. 중심선으로 끌어오는 위치 보정은 없습니다.

## 준비 사항

- Python **3.12 이상**과 Git. Windows는 64비트 Python을 사용합니다.
- 화면을 띄우려면 OpenGL을 지원하는 그래픽 환경이 필요합니다.
- 의존성: `requirements.txt`의 MuJoCo와 NumPy. MuJoCo 엔진은 Python 패키지에 포함됩니다.
- Panda XML과 mesh가 `models/`에 포함돼 있으므로 Menagerie를 별도로 clone할 필요가 없습니다.

macOS의 Python 3.14 환경에서 실행을 확인했습니다. Windows 실행 명령은 아래에 제공하며, Windows 실기기 검증은 아직 하지 않았습니다.

저장소를 내려받은 뒤 **프로젝트 루트**에서 아래 명령을 실행합니다.

```bash
git clone https://github.com/dijaidy/grasping_on_time.git
cd grasping_on_time
```

## macOS 설치 및 실행

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

# 직선 컨베이어
sh run.sh --config experiments/straight_constant.json

# 원형 컨베이어
sh run.sh --config experiments/circular_constant.json

# 둥근 정사각형 컨베이어
sh run.sh --config experiments/rounded_square_constant.json
```

macOS의 passive viewer는 `mjpython`으로 실행해야 합니다. `run.sh`는 프로젝트의 `.venv`에 설치된 `mjpython`을 호출하며, 긴 경로·한글 경로에서 직접 실행 시 발생할 수 있는 launcher 오류도 피합니다. 가상환경 활성화는 필요하지 않습니다. [MuJoCo Python 문서](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)

## Windows 설치 및 실행

PowerShell에서 실행합니다. 아래 예시는 Python 3.12를 설치한 경우입니다. 다른 지원 버전을 사용한다면 `-3.12`를 해당 버전으로 바꿉니다.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

# 직선 컨베이어
.\.venv\Scripts\python.exe simulate.py --config experiments/straight_constant.json

# 원형 컨베이어
.\.venv\Scripts\python.exe simulate.py --config experiments/circular_constant.json

# 둥근 정사각형 컨베이어
.\.venv\Scripts\python.exe simulate.py --config experiments/rounded_square_constant.json
```

Windows에서는 `run.sh`나 `mjpython` 대신 가상환경의 `python.exe`로 실행합니다. 활성화 스크립트를 사용하지 않으므로 PowerShell 실행 정책을 변경할 필요가 없습니다. `py` 명령이 없다면 설치한 Python 3.12 이상의 `python` 명령으로 가상환경을 생성합니다.

## 조작 및 실행 옵션

- **스페이스바:** 로봇, 상자, 시뮬레이션 시간과 컨트롤러 상태를 초기화합니다.
- 창 닫기: 시뮬레이션을 종료합니다.

| 옵션 | 의미 |
| --- | --- |
| `--config 경로` | 실험 JSON 선택. 기본값은 직선 컨베이어 설정 |
| `--no-grasp` | 파지와 파지를 위한 벨트 정지를 끄고 운반만 실행 |
| `--headless` | 뷰어 없이 물리 계산 실행 |
| `--duration 12` | headless 실행의 시뮬레이션 시간. 기본 12초 |

macOS에서 벨트 이동만 확인:

```bash
sh run.sh --config experiments/circular_constant.json --no-grasp
```

Windows에서는 위 옵션을 `.\.venv\Scripts\python.exe simulate.py` 뒤에 동일하게 붙입니다.

화면 없이 파지 검증:

```bash
# macOS
.venv/bin/python simulate.py --config experiments/circular_constant.json --headless --duration 12
```

```powershell
# Windows
.\.venv\Scripts\python.exe simulate.py --config experiments/circular_constant.json --headless --duration 12
```

파지 성공 시 출력의 `phase`가 `success`가 됩니다. 성공 조건은 상자를 초기 위치보다 10cm 이상 들어 올리고 양쪽 손가락 접촉을 0.5초 유지하는 것입니다. 파지를 켠 headless 실행에서 종료 시 성공하지 못했거나 물리 경고가 있으면 종료 코드 1을 반환합니다. `--duration`을 너무 짧게 설정해도 파지가 끝나지 않아 실패로 종료할 수 있습니다.

## 폴더 구조와 코드 역할

```text
grasping_on_time/
├── simulate.py                 # JSON 로딩, 모델 조합, 실행 루프와 초기화
├── scene.xml                   # 기본 직선 장면 및 다른 벨트 조합용 공통 템플릿
├── run.sh                      # macOS용 실행 launcher
├── requirements.txt            # Python 의존성
├── controllers/
│   ├── __init__.py             # 컨트롤러 패키지
│   ├── config.py               # JSON 로딩과 설정값 검증
│   ├── conveyor.py             # 상자 초기화, 접촉 판정, 벨트 방향과 PI 운반력 제어
│   └── ik.py                   # 기본 자세 IK와 정지 물체 접근·파지·상승 제어
├── experiments/
│   ├── straight_constant.json
│   ├── circular_constant.json
│   └── rounded_square_constant.json
├── models/
│   ├── franka_emika_panda/      # Panda XML, mesh, 원본 라이선스
│   ├── conveyors/
│   │   ├── straight.xml
│   │   ├── circular.xml
│   │   └── rounded_square.xml
│   └── objects/box.xml          # 4cm, 100g 동적 상자
└── scripts/
    └── generate_conveyors.py    # 원형·둥근 사각형 XML을 수식으로 재생성
```

`simulate.py`는 JSON에 따라 `scene.xml`의 모델 경로와 배치 좌표를 메모리에서 변경합니다. 원본 `scene.xml` 파일은 덮어쓰지 않습니다. `.venv/`, 원본 `mujoco_menagerie/`, 캐시와 `outputs/`는 Git에서 제외합니다.

## 실험 설정

원형 컨베이어의 예시:

```json
{
  "conveyor": "circular",
  "motion": "constant",
  "speed": [0.1],
  "milestone": [],
  "conveyor_position": [0.62, 0, 0],
  "box_position": [0.39, 0, 0.272],
  "grasp": {
    "enabled": true,
    "stop_time": 0.8,
    "settle_time": 1.0,
    "approach_height": 0.14,
    "lift_height": 0.15
  }
}
```

- 위치는 월드 좌표 `[x, y, z]`이며 단위는 m입니다. `box_position`은 상자 중심입니다.
- `speed`는 경로 진행 방향의 목표 속도(m/s) 배열입니다. 양수는 직선에서 +Y, 폐곡선에서는 위에서 볼 때 반시계 방향입니다. 음수는 역방향입니다.
- `constant`: 속도 하나와 빈 `milestone` 배열을 사용합니다.
- `discrete`: 각 milestone 시각부터 해당 속도로 변경합니다.
- `continuous`: milestone 사이의 속도를 선형 보간합니다.
- 가변 모드에서는 `speed`와 `milestone` 길이가 같아야 하고, 시각은 0부터 시작해 증가해야 합니다. 예: `speed: [0.1, 0.2, 0.05]`, `milestone: [0, 2, 5]`. 마지막 시각 이후에는 마지막 속도를 유지합니다.
- `grasp.enabled`: 자동 파지 여부. 기본 실험은 `stop_time`에 목표 벨트 속도를 0으로 바꾸고, `settle_time` 이상 기다린 뒤 접촉·정지 상태를 확인합니다. 파지 이후 벨트는 초기화 전까지 정지합니다.
- `approach_height`, `lift_height`: 측정한 상자 중심을 기준으로 한 TCP 접근·상승 높이입니다. 처음부터 정지한 물체를 실험하려면 `speed`를 `[0.0]`으로 설정합니다.

## 모델 출처

Panda 모델은 [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)의 `franka_emika_panda`를 사용합니다. 해당 모델의 [Apache 2.0 라이선스](models/franka_emika_panda/LICENSE)를 유지했습니다. 로컬 변경은 파지 기준점인 `tcp` site 추가입니다.
