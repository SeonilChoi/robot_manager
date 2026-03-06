# Changelog

## [Unreleased] – 상태 정리 (홈 테스트 완료 후)

### 정리 및 개선
- **불필요 코드 제거**: `little_reader` 디버그 `print` 제거, 주석 처리된 코드 정리.
- **control/update 일관성**: 궤적 중에는 `control()`에서 내보내는 명령으로 `_current_joint_state` 동기화; `update()`에서는 계획이 없을 때만 `status`로 갱신해 한 스텝에 목표값으로 튀는 현상 방지.
- **Stop 동작**: Stop 시 현재 궤적 위치 스냅샷(`_hold_joint_position`) 저장 후, 리셋 후에는 해당 위치 유지; 새 계획 시작 시 hold 해제.
- **복사 안전성**: Planner `eval_config()` 반환값에 `.copy()` 적용, `generate_trajectory()`의 start/goal config `.copy()`, `little_reader`에서 planner config 수신 시 `.copy()`로 참조 공유 방지.
- **타입**: `Robot.control(status: JointState)` 시그니처를 추상 메서드에 맞춤; `LittleReader.control(status: JointState)` 타입 명시.
- **stop() 중복 제거**: `stop()` 내부에서 `_planner.reset()` 한 번만 호출.

### 네이밍 규칙
- **내부(비공개)**: 앞에 `_` 사용 (예: `_current_joint_state`, `_planner`, `_run()`).
- **공개 API**: `_` 없음 (예: `control()`, `update()`, `plan()`, `is_planned()`, `reset()`).

### GUI
- 1행 2열: 왼쪽 3D 로봇/경로, 오른쪽 조인트별 현재·목표 각도 실시간 그래프.
- Figure 크기 2배 (14×12), 조인트 히스토리 500 스텝.

### 테스트
- 임의 홈으로 가는 시나리오까지 동작 확인 완료.
