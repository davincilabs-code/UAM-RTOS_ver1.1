# UAM-RTOS ver1.1 (Linux demo build)


## 구성
- `kernel/` : RTOS 마이크로커널 (ucontext 기반 태스크 컨텍스트, tick, ready/delay 리스트)
- `sync/`   : Queue / EventFlags / Mutex 
- `app/`    : UAM 데모 태스크 (Sensor/Estimator/Control/Comms/Health/Logger)

## 빠른 실행 (Ubuntu/WSL/Linux)
```bash
sudo apt-get update
sudo apt-get install -y cmake build-essential

mkdir -p build
cmake -S . -B build
cmake --build build -j
./build/uam_rtos_demo
```

## 참고
- 스케줄러는 우선순위 기반이며 tick 인터럽트가 발생해도 즉시 강제 선점(swapcontext)은 하지 않고,
  커널 API 경계(Delay/Queue/Event/Mutex/Yield)에서 reschedule을 수행합니다.
  (데모 안정성을 위해 async-signal-safe 제약을 회피)

## 라이선스
MIT License 
