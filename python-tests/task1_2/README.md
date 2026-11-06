# Task 1.2 (PID tuning) – rosbag 기록 / 비교 도구

체크포인트 1 Task 1.2 리포트용 데이터 수집 스크립트. plot은 Foxglove에서 bag을 열어 직접 그린다.

> 이 펌웨어는 문서와 달리 `~/mbot_firmware/mbot_firmware_ros/` 에 있다.
> 명령은 모두 **Pi에서**, ROS가 source된 셸(기본 `.bashrc`)에서 실행.

## 파일

| 파일 | 역할 |
|---|---|
| `step_test_record.py` | control_mode 설정 → `ros2 bag record`(mcap) → `/cmd_vel` step 시퀀스 송신 → 종료. bag 폴더에 `run_info.json`(파라미터 dump 포함) 저장 |
| `merge_for_compare.py` | FF bag과 tuned bag을 **한 bag**으로 합침. 토픽에 `/ff/...`, `/tuned/...` 접두사, step 시작 시각을 정렬 → Foxglove 한 plot에 overlay 가능 |
| `analyze_step.py` | bag에서 rise time / overshoot / settling / 정상상태 오차 표 출력 (`--csv`, `--png` 옵션) |
| `bag_tools.py` | 위 두 스크립트가 쓰는 공용 헬퍼 |

## 기록되는 토픽

| 토픽 | 내용 | 비고 |
|---|---|---|
| `/cmd_vel` | 명령 (Twist) | step_cmd_test.py와 동일한 시퀀스 |
| `/odom` | 펌웨어 odometry. `twist.twist.linear.x`, `angular.z` = 엔코더 기반 실제 body 속도 | 약 22 Hz |
| `/motor_vel` | 바퀴 각속도 `velocity[0]`=L, `[1]`=R [rad/s] | 약 90 Hz |
| `/encoders`, `/imu` | 원본 센서 | |
| `/step_test/cmd` | 명령 (TwistStamped, 0 포함 깨끗한 step) | 스크립트가 발행 |
| `/step_test/cmd_wheel` | 명령을 펌웨어와 같은 kinematics로 바꾼 바퀴 목표 각속도 | `velocity[0]`=L, `[1]`=R |
| `/step_test/meas` | `/motor_vel`로 계산한 body 속도 (vx, wz). `/odom`보다 4배 촘촘 | 스크립트가 발행 |

Foxglove에서 가장 간단한 조합: `/step_test/cmd.twist.linear.x` vs `/odom.twist.twist.linear.x` (또는 `/step_test/meas.twist.linear.x`).
바퀴 단위로 보려면 `/step_test/cmd_wheel.velocity[0]` vs `/motor_vel.velocity[0]`.

## 실험 절차

```bash
cd ~/mbot_firmware/mbot_firmware_ros/python-tests/task1_2
```

1. **FF only** (control_mode 0) – 로봇을 바닥에 놓고:
   ```bash
   python3 step_test_record.py ff --label ff
   ```
2. **튜닝한 컨트롤러** – `include/config/mbot_classic_pid.yaml` 수정 후 (`--load-yaml`이 `ros2 param load`를 대신 실행):
   ```bash
   python3 step_test_record.py ffpid --label tuned --load-yaml
   ```
   PID only로 비교하려면 `pid`. 게인만 바꿔가며 여러 번 돌릴 때는 `--label kp0.15` 처럼 이름을 붙인다.
3. bag은 `~/bags/task1_2/<label>_<날짜시각>/` 에 생긴다.

시퀀스(기본): 정지 1.5 s → **vx 0.5 m/s 4 s** → 정지 1 s → wz −1 rad/s 4 s → 정지 1 s → wz +1 rad/s 4 s → 정지 1 s.
옵션: `--vx`, `--wz`, `--hold`, `--no-rotate` (직진 step만), `--out-root`.
`Ctrl+C` 하면 즉시 0 명령을 보내고 bag을 닫는다.

## FF vs tuned 를 한 plot에 겹치기

```bash
python3 merge_for_compare.py ~/bags/task1_2/ff_* ~/bags/task1_2/tuned_* --labels ff tuned -o ~/bags/task1_2/compare
```
→ `compare/` bag 하나를 Foxglove에서 열고 `/ff/odom.twist.twist.linear.x`, `/tuned/odom.twist.twist.linear.x`, `/ff/step_test/cmd.twist.linear.x` 를 같은 plot에 올린다.
두 run의 step 시작이 같은 시각으로 정렬돼 있다 (Foxglove plot의 timestamp는 기본값인 *receive time* 이어야 함).

## 수치 지표 (리포트 표용)

```bash
python3 analyze_step.py ~/bags/task1_2/ff_* ~/bags/task1_2/tuned_* --labels FF "FF+PID" --csv metrics.csv --png step.png
```
세그먼트(직진 0.5 m/s, CW, CCW)마다 10–90 % rise time, overshoot, 5 % settling time, 정상상태 평균/오차, RMS 오차를 출력한다.
`--source meas` 로 하면 `/odom` 대신 `/step_test/meas`(90 Hz)를 쓴다. `--png`는 Foxglove 대신 쓸 수 있는 백업 그림.

## Foxglove로 bag 열기

- **파일로**: 노트북에서 `scp -r mbot@<pi-ip>:~/bags/task1_2/<bag> .` 후 Foxglove → Open local file → `*.mcap`.
- **실시간**: Pi에서 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` 띄우고 Foxglove에서 `ws://<pi-ip>:8765` 연결. 이 경우 `step_test_record.py`를 그대로 돌리면 되고, 위의 `/step_test/*` 토픽이 실시간으로도 보인다.

## 주의

- 실험 전 `ros2 param dump /mbot_control_node` 로 게인 확인. 2026-11-04 현재 보드에는 기본값(kp 0.1, kd 0.01, tf 0.1, mode 2)이 들어 있고 yaml(kp 0.128, tf 0.275)은 아직 로드되지 않은 상태였다.
- 캘리브레이션 펌웨어를 다시 flash하면 게인이 `mbot_classic_default_pid.h` 값으로 덮어써진다 → `--load-yaml` 다시 실행.
- 스크립트는 종료 후 control_mode를 되돌리지 않는다. FF run 뒤에는 `ros2 param set /mbot_control_node control_mode 2`.
- `tf`는 `dt/2 = 0.005` 보다 커야 한다 (`rc_filter_pid`가 거부). PID 출력은 PWM 단위로 ±1에 포화되고 FF PWM에 더해진다.
- 펌웨어 PWM 1차 저역통과 필터(시정수 0.2 s, `enable_pwm_lpf`)가 켜져 있어 어떤 모드든 응답에 이 지연이 포함된다.
- 명령이 1 s 이상 끊기면 펌웨어가 모터를 0으로 만든다 (스크립트는 50 Hz로 계속 보냄).
