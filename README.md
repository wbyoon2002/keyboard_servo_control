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

| 키 | 동작 |
|----|------|
| `q w e r t` | 모터 0·1·2·3·4 **시계방향**(각도 증가) |
| `a s d f g` | 모터 0·1·2·3·4 **반시계방향**(각도 감소) |
| 여러 키 동시에 | 해당 모터들이 **동시에** 회전 |
| `=` / `+` | 회전 속도 ↑ |
| `-` | 회전 속도 ↓ |
| `Space` | 현재 모든 모터 각도를 **새 파일로 저장** |
| `h` | 모든 모터를 중앙(90°)으로 |
| `o` | 모든 모터 출력 해제(토크 off) |
| `Esc` | 종료(종료 시 자동으로 출력 해제) |

- 키를 **누르고 있는 동안** 설정된 속도(deg/s)로 계속 회전합니다.
- 같은 모터의 시계/반시계 키를 동시에 누르면 상쇄되어 멈춥니다.

## 5. 저장 파일

`Space`를 누를 때마다 `saved_angles/angles_YYYYMMDD_HHMMSS.txt` 형식의
**새 파일**이 생성됩니다. 각 모터의 이름도 함께 기록됩니다. 내용 예시:

```
# Saved motor angles
# timestamp : 2026-06-16T14:32:10
# speed     : 60 deg/s
motor 0 (Base    ) : 95 deg
motor 1 (Shoulder) : 88 deg
motor 2 (Elbow   ) : 90 deg
motor 3 (Wrist   ) : 120 deg
motor 4 (Gripper ) : 60 deg

names  = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]
angles = [95, 88, 90, 120, 60]
S 95 88 90 120 60
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
