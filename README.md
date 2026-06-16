# 키보드 모터 제어 (Keyboard Motor Control)

5개의 서보를 키보드로 회전시키고, 현재 각도를 파일로 저장하는 시스템입니다.

- **펌웨어**: [`firmware/keyboard_motor_control/keyboard_motor_control.ino`](./firmware/keyboard_motor_control/keyboard_motor_control.ino)
- **PC 제어**: [`motor_keyboard_control.py`](./motor_keyboard_control.py)

PC가 속도/위치 계산을 모두 담당하고, 아두이노에는 매 프레임 5개 모터의 절대 각도만
스트리밍합니다. 아두이노는 받은 각도를 `../arduino_motor_control/calibration_results.txt`의
보정값으로 펄스폭(µs)으로 변환해 PCA9685에 출력하는 얇은 액추에이터 역할만 합니다.

## 1. 준비

### 라이브러리 설치 (PC)

```powershell
pip install -r requirements.txt
```

(`pyserial` + `keyboard`)

> `keyboard` 라이브러리는 실제 하드웨어 키 상태를 직접 읽기 때문에 **여러 키를 동시에
> 눌러 여러 모터를 동시에** 돌릴 수 있습니다. 키 입력이 안 잡히면 터미널을
> **관리자 권한**으로 다시 실행하세요.

### 펌웨어 업로드 (아두이노)

```powershell
& 'C:\Users\wonbe\bin\arduino-cli.exe' compile --fqbn arduino:avr:uno firmware/keyboard_motor_control
& 'C:\Users\wonbe\bin\arduino-cli.exe' upload -p COMx --fqbn arduino:avr:uno firmware/keyboard_motor_control
```

`COMx`는 실제 포트로 바꾸세요. 펌웨어는 **115200 baud**를 사용합니다.

## 2. 실행

```powershell
python motor_keyboard_control.py            # 기본 포트 COM3
python motor_keyboard_control.py COM5       # 포트 직접 지정
```

## 3. 모터 이름 지정

`motor_keyboard_control.py` 상단의 `MOTOR_NAMES`를 편집하면 각 모터에 이름을
붙일 수 있습니다(순서는 모터 0~4).

```python
MOTOR_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]
```

이름은 화면 표시와 저장 파일에만 쓰이고 펌웨어는 인덱스(0~4)로 동작하므로,
다른 코드를 건드릴 필요 없이 자유롭게 바꿔도 됩니다. 빈 문자열로 두면 해당
모터는 `motor N`으로 표시됩니다.

## 4. 키 조작

모터별 조작 (앞 키 = 각도 증가, 뒤 키 = 각도 감소):

| 키 | 모터 | 실제 동작 |
|----|------|----------|
| `q` / `a` | arm_top | 시계방향 / 반시계방향 회전 |
| `s` / `w` | arm_wheel | 시계방향 / 반시계방향 회전 |
| `d` / `e` | arm_bottom | 시계방향 / 반시계방향 회전 |
| `f` / `r` | arm_left | 링키지 내림 / 올림 |
| `t` / `g` | arm_right | 링키지 올림 / 내림 |

여러 키를 동시에 누르면 해당 모터들이 **동시에** 회전합니다.

| 키 | 동작 |
|----|------|
| `=` / `+` | 회전 속도 ↑ |
| `-` | 회전 속도 ↓ |
| `1`~`9` | 저장된 세트 `saved_angles/angles_<n>.txt`로 **이동** |
| `l` | 사용 가능한 저장 세트 목록 출력 |
| `Space` | 현재 모든 모터 각도를 **새 파일로 저장** |
| `h` | 모든 모터를 중앙(90°)으로 |
| `o` | 모든 모터 출력 해제(토크 off) |
| `Esc` | 종료(종료 시 자동으로 출력 해제) |

- 키를 **누르고 있는 동안** 설정된 속도(deg/s)로 계속 회전합니다.
- 같은 모터의 시계/반시계 키를 동시에 누르면 상쇄되어 멈춥니다.

### 저장된 각도 세트 불러오기

제어 중에 숫자 키 `1`~`9`를 누르면 `saved_angles/angles_<숫자>.txt` 파일을 읽어
모든 모터를 그 자세로 **현재 속도(deg/s)에 맞춰 부드럽게 이동**시킵니다.

- 예: `2`를 누르면 `saved_angles/angles_2.txt`의 각도로 이동.
- 이동 중 아무 모터 키나 누르면 이동이 **즉시 취소**되고 수동 제어로 돌아갑니다.
- `h`(중앙) 또는 `o`(출력 해제)를 누르면 이동이 취소됩니다.
- 상태 줄에 `[-> set]` 표시가 나오면 자동 이동 중이라는 뜻입니다.
- `l`을 누르면 현재 사용 가능한 세트 번호와 각도를 표로 보여줍니다.

세트 파일은 `Space`로 저장한 파일의 이름을 `angles_1.txt`, `angles_2.txt` … 처럼
**숫자로 바꿔** 두면 됩니다(파일 형식은 그대로 사용).

## 5. 저장 파일

`Space`를 누를 때마다 `saved_angles/angles_YYYYMMDD_HHMMSS.txt` 형식의
**새 파일**이 생성됩니다. 각 모터의 이름도 함께 기록됩니다. 내용 예시:

```
# Saved motor angles
# timestamp : 2026-06-16T18:37:13
# speed     : 60 deg/s
motor 0 (arm_top   ) : 90 deg
motor 1 (arm_wheel ) : 180 deg
motor 2 (arm_bottom) : 0 deg
motor 3 (arm_left  ) : 63 deg
motor 4 (arm_right ) : 43 deg

names  = ["arm_top", "arm_wheel", "arm_bottom", "arm_left", "arm_right"]
angles = [90, 180, 0, 63, 43]
S 90 180 0 63 43
```

마지막 `S ...` 줄은 그대로 아두이노에 보내면 같은 자세를 재현할 수 있는 명령입니다.

## 시리얼 프로토콜 (참고)

| 명령 | 의미 |
|------|------|
| `S a0 a1 a2 a3 a4` | 5개 서보를 각 각도(0–180°)로 |
| `M i a` | 서보 `i`(0–4)를 각도 `a`로 |
| `H` | 전체 중앙(90°) |
| `O` | 전체 출력 해제 |
| `Q` | 현재 각도 응답(`A a0 a1 a2 a3 a4`) |
| `P` | 적용 중인 보정 배열 출력 |

## 회전 방향이 반대일 때

`q/a`로 도는 물리적 방향이 원하는 시계/반시계와 반대라면, 서보 혼을 반대로 끼우거나
`motor_keyboard_control.py`의 `CW_KEYS`/`CCW_KEYS` 매핑을 서로 바꾸면 됩니다.
