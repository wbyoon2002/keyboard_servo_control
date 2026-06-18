# 키보드 / 터미널 모터 제어 (Keyboard & Terminal Motor Control)

5개의 서보를 PC에서 제어하고, 현재 각도를 파일로 저장하거나 매크로 동작을
실행하는 시스템입니다.

- **펌웨어**: [`firmware/keyboard_motor_control/keyboard_motor_control.ino`](./firmware/keyboard_motor_control/keyboard_motor_control.ino)
- **PC 제어 — 화면 버튼(GUI)**: [`gui_motor_control.py`](./gui_motor_control.py)
- **PC 제어 — 실시간 키보드**: [`motor_keyboard_control.py`](./motor_keyboard_control.py)
- **PC 제어 — 터미널 명령 / 매크로**: [`terminal_control.py`](./terminal_control.py)

## 동작 방식

PC는 모터 인덱스(0~4)와 각도(필요하면 속도)만 보내고, 아두이노가 각 모터의
보정값(`../arduino_motor_control/calibration_results.txt`)으로 펄스폭(µs)을 계산해
PCA9685에 출력합니다. 두 가지 이동 방식을 지원합니다.

- **즉시 이동** — `S`(전체) / `M i a`(단일, 속도 없음) 명령. 받은 각도를 그대로
  바로 출력합니다. PC가 매 프레임 절대 각도를 스트리밍하는 **키보드 실시간
  제어**가 여기에 해당합니다.
- **속도 보간 이동** — `M i a v`처럼 속도(deg/s)를 함께 보내면, 아두이노가
  ~100Hz로 목표 각도까지 **부드럽게 보간**합니다. 목표 근처(15°)에서는
  감속(ease-out)하고 최소 속도(8 deg/s)를 유지해 종단 떨림·멈춤을 막습니다.

즉, 펌웨어는 단순 액추에이터를 넘어 자체적으로 속도 제어 궤적을 생성하므로,
PC는 "어디로, 얼마나 빠르게"만 지정하면 됩니다.

## 1. 준비

### 라이브러리 설치 (PC)

```powershell
pip install -r requirements.txt
```

(`pyserial` + `keyboard`)

> `keyboard` 라이브러리는 실제 하드웨어 키 상태를 직접 읽기 때문에 **여러 키를 동시에
> 눌러 여러 모터를 동시에** 돌릴 수 있습니다. 키 입력이 안 잡히면 터미널을
> **관리자 권한**으로 다시 실행하세요. (터미널 제어 `terminal_control.py`는
> `pyserial`만 사용하므로 관리자 권한이 필요 없습니다.)

### 펌웨어 업로드 (아두이노)

```powershell
& 'C:\Users\wonbe\bin\arduino-cli.exe' compile --fqbn arduino:avr:uno firmware/keyboard_motor_control
& 'C:\Users\wonbe\bin\arduino-cli.exe' upload -p COMx --fqbn arduino:avr:uno firmware/keyboard_motor_control
```

`COMx`는 실제 포트로 바꾸세요. 펌웨어는 **115200 baud**를 사용합니다.

## 2. 화면 버튼(GUI) 제어 (`gui_motor_control.py`)

키보드 대신 **화면의 버튼·스핀박스**로 모터를 제어하는 도구입니다. 여러 모터를
**동시에** 움직일 수 있습니다. `pyserial`만 있으면 되고(Tkinter는 파이썬에
기본 포함), `keyboard` 라이브러리나 **관리자 권한이 필요 없습니다**.

```powershell
python gui_motor_control.py            # 기본 포트 COM3
python gui_motor_control.py COM5       # 포트 직접 지정
```

### 화면 구성과 조작

1. 상단 **Port** 칸에 포트를 입력하고 **Connect**를 누릅니다. 연결되면 약 2초
   뒤 자동으로 홈(90°)으로 이동합니다.
2. 모터 표에서 제어할 모터를 **체크(On)**하고, 각 모터의 **방향(+/−)**과
   **속도(deg/s)**, 필요하면 **목표 각도(Target)**를 설정합니다.
3. 움직이는 방법은 두 가지입니다.
   - **HOLD TO MOVE**(초록 버튼)를 **누르고 있는 동안**, 선택한 모든 모터가
     각자의 방향·속도로 **동시에** 회전합니다. 떼면 멈춥니다.
   - **GO TO TARGET**(파란 버튼)을 누르면, 선택한 모터들이 각자의 **목표
     각도로 동시에** 이동합니다(보간은 아두이노가 처리).
4. 모터별 **+ / −** 버튼은 해당 모터만 `5°`씩 살짝 움직이는 미세 조정입니다.

| 버튼 | 동작 |
|------|------|
| `HOLD TO MOVE` | 선택 모터를 방향·속도대로 **동시에** 회전(누르는 동안) |
| `GO TO TARGET` | 선택 모터를 목표 각도로 **동시에** 이동 |
| `STOP / HOLD` | 현재 위치에서 모두 정지·유지 |
| `Home (90°)` | 전체 중앙으로 |
| `Release` | 전체 출력 해제(토크 off) |
| `Query angles` | 현재 각도 조회(표시값 동기화) |
| `Select All/None` | 모든 모터 선택/해제 |

`+` 방향의 실제 동작(모터 0~4)은 각 행 옆에 표시됩니다
(`+CW/−CCW`, `arm_left`는 `+내림/−올림`, `arm_right`는 `+내림/−올림`).
하단 **Serial log**에 아두이노로 보낸 명령과 응답(`OK`/`A`/`ERR`)이 표시됩니다.

### 자세 저장·불러오기 (이름 지정)

**Save / recall poses by name** 영역에서 현재 자세를 **이름을 붙여** 저장하고,
그 이름으로 다시 불러올 수 있습니다.

- **저장**: `Name` 칸에 원하는 이름(예: `ready`, `grab pose`)을 입력하고
  **Save pose**를 누릅니다(`Enter`로도 저장). 같은 이름으로 다시 저장하면
  **덮어쓰기**됩니다. 이름을 비워 두면 기존처럼 타임스탬프 파일로 저장됩니다.
- **불러오기**: `Saved` 드롭다운에서 이름을 고르고 **Move there**를 누르면
  (또는 항목을 더블클릭하면) 저장된 자세로 부드럽게 이동합니다.
- **삭제**: 드롭다운에서 고른 자세를 **Delete**로 지웁니다. 목록은
  **Refresh**로 새로 고칩니다.

파일은 `saved_angles/angles_<이름>.txt`로 저장되며(공백·특수문자는 파일명에서
정리되지만 표시 이름은 파일 안 `# name :` 줄에 그대로 보존), 형식은 키보드
도구와 동일합니다(아래 "저장 파일" 참고). 기존에 저장해 둔 `angles_1.txt` 같은
파일도 드롭다운에 그대로 나타나 불러올 수 있습니다.

### 매크로 저장·불러오기·편집

상단 **Macros…** 버튼을 누르면 매크로 편집 창이 열립니다. `terminal_control.py`와
**동일한 형식**(스텝 타입 `move` / `relative` / `set_all` / `delay` / `home` /
`release` / `run`)의 동작 시퀀스를 만들어 실행할 수 있습니다. (수동 조작을
녹화하는 방식이 아니라, 스텝을 직접 추가·정렬·수정하는 방식입니다.)

- **매크로 목록**(좌측): `New`(새로 만들기) · `Rename`(이름 변경) ·
  `Dup`(복제) · `Delete`(삭제). 매크로를 고르면 우측에 스텝이 표시됩니다.
- **스텝 편집**(우측): 아래 폼에서 `Type`을 고르면 필요한 입력칸(모터 인덱스,
  각도, 속도, 초, set_all 각도 5개, run 대상 매크로)만 나타납니다.
  - **Add step**(맨 끝에 추가) / **Insert above**(선택 위에 삽입) /
    **Update selected**(선택 스텝 덮어쓰기).
  - 스텝을 선택하면 그 값이 폼에 채워져 바로 수정할 수 있습니다.
  - **↑ Up / ↓ Down**으로 **순서 변경**, **Delete step**으로 삭제.
- **실행**: **▶ Run**으로 선택한 매크로를 실행, **■ Stop**으로 중단합니다.
  실행 중 본창의 `STOP / HOLD`·`Home`·`Release`를 누르거나 **MOVE**를 잡으면
  매크로가 자동으로 중단되고 수동 제어로 돌아갑니다.
- **저장된 자세 참고**(창 하단의 `Saved poses` 패널): `saved_angles/`에 저장해
  둔 자세 이름과 **각도값(논리 각도)**이 표로 표시됩니다. 자세를 고른 뒤
  **Use in set_all field**를 누르면 그 각도가 `set_all` 입력칸에 채워져 바로
  스텝으로 추가할 수 있고, **Refresh**로 목록을 새로 고칩니다(본창에서 자세를
  저장/삭제하면 자동으로 갱신).

| 스텝 타입 | 파라미터 | 설명 |
|-----------|----------|------|
| `move` | `index`, `angle`, `speed` | 모터를 **절대 각도**로 이동(`speed` 0 = 즉시) |
| `relative` | `index`, `angle`, `speed` | 현재 위치에서 **상대 이동**(각도는 ±) |
| `set_all` | `angles`(5개) | 5개 모터를 각도 배열로 **즉시** 설정 |
| `delay` | `seconds` | 지정 시간(초) 대기 |
| `home` | – | 전체 중앙(90°) |
| `release` | – | 전체 출력 해제 |
| `run` | `macro` | 다른 매크로를 **중첩 호출**(순환은 자동 차단) |

매크로는 `macros.json`에 **자동 저장**되어 다음 실행 때도 유지됩니다. 각도는
GUI와 동일한 **논리 각도(0–180)** 기준이며, 시리얼로 보낼 때 모터별 보정
(`INVERT`·`OFFSET`)이 자동 적용됩니다. `delay`는 GUI를 멈추지 않고(논블로킹)
처리되며, 모터 이동의 실제 타이밍은 직접 넣은 `delay` 스텝으로 조절합니다
(이동 보간은 아두이노가 처리).

#### 각도 보정 (`INVERT` / `OFFSET`)

`gui_motor_control.py` 상단의 두 배열로 **논리 각도 → 물리 각도**를 보정합니다.

```python
physical = (INVERT[i] ? 180 - logical : logical) + OFFSET[i]
```

- `INVERT[i]` : 방향 반전(반대로 도는 모터 1·2·4). 단순 오프셋으로는 방향을
  뒤집을 수 없으므로 **방향 보정은 이 값**으로 합니다.
- `OFFSET[i]` : 영점 보정(도 단위). `terminal_control.py`의 `AngleOffsetManager`
  (`physical = logical + offset`)와 같은 역할입니다. 현재 `OFFSET[0] = -10`으로
  모터 0의 영점을 `terminal_control.py`와 맞춰 두었고(나머지는 0), 빌드에 맞게
  자유롭게 조정하면 됩니다.

> 처음 실행하면 `terminal_control.py`의 매크로(`aleft_hold`·`aright_hold`·
> `roll_home`·`roll_right`·`flip_right`·`roll` 등)와 예시 `demo_sequence`가 들어
> 있습니다. GUI는 모터 1·2·4를 `INVERT`로 반전하지만 `terminal_control.py`는
> 그렇지 않으므로, **모터 1·2·4의 값은 같은 물리 동작이 나오도록 변환**해
> 두었습니다(`move`: `180 - 각도`, `relative`: 부호 반전). 모터 0·3 값은 그대로이며
> 모터 0 영점은 `OFFSET[0]`로 맞춥니다. 동작이 다르면 `INVERT`·`OFFSET`을
> 조정하거나 스텝 값을 직접 편집하세요. 필요 없는 매크로는 `Delete`로 지우면
> 됩니다(`macros.json`을 직접 편집해도 됩니다).

## 3. 키보드 실시간 제어 (`motor_keyboard_control.py`)

```powershell
python motor_keyboard_control.py            # 기본 포트 COM3
python motor_keyboard_control.py COM5       # 포트 직접 지정
```

### 모터 이름 지정

`motor_keyboard_control.py` 상단의 `MOTOR_NAMES`를 편집하면 각 모터에 이름을
붙일 수 있습니다(순서는 모터 0~4).

```python
MOTOR_NAMES = ["arm_top", "arm_wheel", "arm_bottom", "arm_left", "arm_right"]
```

이름은 화면 표시와 저장 파일에만 쓰이고 펌웨어는 인덱스(0~4)로 동작하므로,
다른 코드를 건드릴 필요 없이 자유롭게 바꿔도 됩니다. 빈 문자열로 두면 해당
모터는 `motor N`으로 표시됩니다.

### 키 조작

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
- 수동 조작 중에는 PC가 매 프레임 절대 각도(`S`)를 스트리밍합니다.

### 저장된 각도 세트 불러오기

제어 중에 숫자 키 `1`~`9`를 누르면 `saved_angles/angles_<숫자>.txt` 파일을 읽어
모든 모터를 그 자세로 **현재 속도(deg/s)에 맞춰 부드럽게 이동**시킵니다. 이동
명령(`M i a v`)을 각 모터에 보내면 보간은 **아두이노가 직접** 처리합니다.

- 예: `2`를 누르면 `saved_angles/angles_2.txt`의 각도로 이동.
- 이동 중 아무 모터 키나 누르면 이동이 **즉시 취소**되고 수동 제어로 돌아갑니다.
- `h`(중앙) 또는 `o`(출력 해제)를 누르면 이동이 취소됩니다.
- 상태 줄에 `[-> set]` 표시가 나오면 자동 이동 중이라는 뜻입니다.
- `l`을 누르면 현재 사용 가능한 세트 번호와 각도를 표로 보여줍니다.

세트 파일은 `Space`로 저장한 파일의 이름을 `angles_1.txt`, `angles_2.txt` … 처럼
**숫자로 바꿔** 두면 됩니다(파일 형식은 그대로 사용).

### 저장 파일

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

## 4. 터미널 명령 / 매크로 제어 (`terminal_control.py`)

키보드 실시간 제어 대신 **명령을 한 줄씩 입력**해 단일/전체 모터를 움직이거나,
미리 정의한 **매크로 시퀀스**를 실행하는 대화형 도구입니다. 속도 보간 이동을
적극적으로 활용하므로 정밀한 동작 시퀀스를 만들기 좋습니다.

```powershell
python terminal_control.py            # 기본 포트 COM3
python terminal_control.py COM5       # 포트 직접 지정
```

실행하면 `ServoCMD>` 프롬프트가 뜹니다. 명령은 다음과 같습니다.

| 명령 | 의미 |
|------|------|
| `M <i> <a> [<v>]` | 모터 `i`(0–4)를 **절대 각도** `a`로 (선택 속도 `v` deg/s) |
| `R <i> <a> [<v>]` | 모터 `i`를 현재 위치에서 **상대 이동** `a`도 (선택 속도 `v`) |
| `S <a0>..<a4>` | 5개 모터를 각 각도로 **즉시** 설정 |
| `H` | 전체 중앙(90°)으로 홈 |
| `O` | 전체 출력 해제(토크 off) |
| `Q` | 현재 각도 조회 (논리 각도로 환산해 출력) |
| `P` | 적용 중인 보정값 출력 |
| `run <매크로>` | 매크로 실행 |
| `list` | 사용 가능한 매크로 목록 |
| `help` / `?` | 도움말 |
| `exit` / `quit` | 종료(종료 시 자동 출력 해제) |

각 명령은 아두이노에 보낸 뒤 응답(`OK ...` / `A ...` / `ERR ...`)을 함께 출력해
동기화 상태를 확인할 수 있습니다.

### 각도 오프셋 보정

`AngleOffsetManager`가 **논리 각도 → 물리 각도** 오프셋을 적용합니다
(`물리 = 논리 + offset`). 예를 들어 모터 0(arm_top)은 논리 90°가 물리 55°에
대응하도록 오프셋 `-35`가 설정되어 있어, 사용자는 항상 일관된 논리 각도로
명령하면 됩니다. 오프셋 값은 `terminal_control.py`의 `AngleOffsetManager`에서
바꿀 수 있습니다.

### 매크로

`MACROS` 딕셔너리에 동작 시퀀스를 정의해 `run <이름>`으로 실행합니다. 각 스텝은
다음 타입을 지원합니다.

| 스텝 타입 | 설명 |
|-----------|------|
| `move` | 단일 모터를 절대 각도로 이동 (`index`, `angle`, 선택 `speed`) |
| `relative` | 단일 모터를 현재 위치에서 상대 이동 (`index`, `angle`, 선택 `speed`) |
| `set_all` | 5개 모터를 각도 배열로 즉시 설정 (`angles`) |
| `home` | 전체 중앙(90°)으로 |
| `release` | 전체 출력 해제 |
| `delay` | 지정 시간(초) 대기 (`seconds`) |
| `run` | 다른 매크로를 중첩 호출 (`macro`) |

예시:

```python
MACROS = {
    "aleft_hold": [
        {"type": "move", "index": ALEFT, "angle": 135, "speed": 10},
    ],
    "roll_right": [
        {"type": "move", "index": ABOT, "angle": 0,   "speed": 15},
        {"type": "move", "index": ATOP, "angle": 135, "speed": 15},
        {"type": "delay", "seconds": 1.0},
        {"type": "relative", "index": ATOP, "angle": 15, "speed": 15},
    ],
}
```

`ATOP / WHEEL / ABOT / ALEFT / ARIGHT` 상수가 모터 인덱스 0~4를 가리킵니다.

## 5. 시리얼 프로토콜 (참고)

| 명령 | 의미 |
|------|------|
| `S a0 a1 a2 a3 a4` | 5개 서보를 각 각도(0–180°)로 **즉시** |
| `M i a` | 서보 `i`(0–4)를 각도 `a`로 **즉시** |
| `M i a v` | 서보 `i`를 각도 `a`로 속도 `v`(deg/s)로 **부드럽게 보간 이동** |
| `H` | 전체 중앙(90°) |
| `O` | 전체 출력 해제 |
| `Q` | 현재 각도 응답(`A a0 a1 a2 a3 a4`) |
| `P` | 적용 중인 보정 배열 출력 |

- 모든 명령은 115200 baud, 줄바꿈(`\n`)으로 끝나며 대소문자를 구분합니다.
- 정상 명령은 `OK <명령>`, 잘못된 명령은 `ERR <사유>`로 응답합니다.
- `M i a v`의 속도 보간은 아두이노에서 ~100Hz로 처리되며, 목표 15° 이내에서
  감속(ease-out)하고 최소 8 deg/s를 유지합니다.

## 회전 방향이 반대일 때

`q/a`로 도는 물리적 방향이 원하는 시계/반시계와 반대라면, 서보 혼을 반대로 끼우거나
`motor_keyboard_control.py`의 `CW_KEYS`/`CCW_KEYS` 매핑을 서로 바꾸면 됩니다.
