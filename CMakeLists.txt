cmake_minimum_required(VERSION 3.16)
project(UAM_RTOS_DEMO C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(uam_rtos_demo
    kernel/rtos.c
    sync/queue.c
    sync/event_flags.c
    sync/mutex.c
    app/main.c
    app/tasks_sensor.c
    app/tasks_estimator.c
    app/tasks_control.c
    app/tasks_comms.c
    app/tasks_health.c
    app/tasks_logger.c
    app/uam_messages.c
)

target_include_directories(uam_rtos_demo PRIVATE
    kernel
    sync
    app
)

# ucontext is in libc; no extra libs typically. Some distros may require -pthread for timers/stdio flush; keep it.
target_link_libraries(uam_rtos_demo PRIVATE pthread m)

target_compile_options(uam_rtos_demo PRIVATE -Wall -Wextra -Wpedantic -O2)
