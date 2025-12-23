# Architecture

## Kernel
- **TCB(Task Control Block)**:
  - ucontext 기반 컨텍스트 저장
  - 고정 우선순위(priority 0~7)
  - ready/delayed 상태 관리
- **Tick**
  - `setitimer(ITIMER_REAL)`로 SIGALRM tick 발생
  - handler에서 tick 증가 + wakeup 처리
  - async-signal-safe 제한 때문에 **즉시 swapcontext(강제 선점)**은 하지 않고,
    커널 API 호출 지점에서 reschedule 수행

## Sync primitives
- Queue: 고정 길이 링버퍼 + 블로킹(간단 폴링/타임아웃)
- EventFlags: bitmask wait-any/wait-all + 타임아웃
- Mutex: 단순 스핀 + 블로킹(폴링/타임아웃)

## App tasks (demo)
- SensorTask (200ms): 센서 샘플 생성
- EstimatorTask (50ms): 저주기 상태추정(예: low-pass/alpha-beta)
- ControlTask (20ms): 제어 입력 계산(PID placeholder)
- CommsTask (100ms): 텔레메트리 패킷 출력(데모)
- LoggerTask (500ms): 시스템 상태 요약 출력
- HealthTask (1s): watchdog/heartbeat 체크

> 실제 시스템 적용 시, 각 태스크는 FACE IO/Transport, ICD 기반 bus message로 자연스럽게 확장 가능.
