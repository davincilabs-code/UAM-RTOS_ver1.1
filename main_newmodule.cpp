// Path: cpp/DavinciFC/main.cpp
// Env: Ubuntu 20.04+, g++ 9.4+, C++17, POSIX
// Build (example):
//   g++ -O2 -std=gnu++17 -pthread -I../cpp -o davinci_fc ../cpp/DavinciFC/main.cpp
// Run:
//   ./davinci_fc [/dev/ttyUSB0] [/dev/ttyUSB1] [can0] [/dev/ttyUSB2] [9999]
// Docs:
//   - Linux SocketCAN: https://docs.kernel.org/networking/can.html
//   - POSIX termios:   https://man7.org/linux/man-pages/man3/termios.3.html
//   - VectorNav SDK:   https://www.vectornav.com/resources

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cstring>
#include <string>
#include <atomic>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

// Ensure PWM sysfs macros/arrays/helpers are available BEFORE tools.h uses them
#include "hal/MTpwm/MTpwm.h"   // provides SYSFS_PWM_DIR, pwm_chips, pwm_channels, write_sysfs
#include "tools.h"
#include <mutex>
#include "VN300/VN300.h"       // init_vn300, processSensorData
#include <termios.h>
#include "fcc.h"               // FCC_Data_Print 안에서 LIHAI까지 함께 출력되도록 수정됨

extern "C" {
#include "UAM_Flight_control.h"
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "rtiostream.h"
FCCData_t g_fcc;
}
static std::mutex g_in_mtx;
static std::mutex g_out_mtx;
#ifndef rtmGetNumSampleTimes
#define rtmGetNumSampleTimes(rtm) 3
#endif

#include "SimulinkGen/fcc_mapper.h"
#include "sbus/SBUS.h"
#include "lihai/lihai.h"       // lihai_open_serial_115200, send_all_packets_serial, read_state_errors_packet20
#include "BMS/bms.h"           // open_can, UpdateBMSData_test
#include "keti/keti.h"
// ==== Safety helpers (add once) ==========================================
#include <cmath>
inline bool finite_d(double v){ return std::isfinite(v); }
inline double clamp_d(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
// yaw unwrap: 이전 프레임과 현재 값 차이가 ±π를 넘으면 연속이 되도록 보정
inline double unwrap_angle(double prev, double now){
    double d = now - prev;
    while (d >  M_PI) { now -= 2*M_PI; d -= 2*M_PI; }
    while (d < -M_PI) { now += 2*M_PI; d += 2*M_PI; }
    return now;
}
// PWM 범위 보조
inline uint16_t clamp_us_u16(double v){
    if (!std::isfinite(v)) return 1000;
    int x = (int)std::lround(v);
    if (x < 1000) x = 1000; else if (x > 2000) x = 2000;
    return (uint16_t)x;
}
// ==== Anomaly probe: timing & event logger =================================
#include <atomic>
#include <chrono>
#include <cstdio>

inline uint64_t now_ns() {
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}
inline double ns_to_ms(uint64_t ns){ return ns / 1e6; }

// 입력원 갱신 타임스탬프/시퀀스
static std::atomic<uint32_t> g_vn_seq{0},   g_sbus_seq{0};
static std::atomic<uint64_t> g_vn_last{0},  g_sbus_last{0};

// 이벤트가 있을 때만 한 줄씩 쓰는 CSV 로거
static FILE* g_anom = nullptr;
static void anom_open_once() {
  if (!g_anom) {
    g_anom = std::fopen("anom_log.csv","w");
    if (g_anom) {
      std::fprintf(g_anom,
        "t_ms,reason,vn_age_ms,sbus_age_ms,vn_seq,sbus_seq,"
        "roll,pitch,yaw,p,q,r,"
        "m0,m1,m2,m3,m4,m5\n");
      std::fflush(g_anom);
    }
  }
}
enum : uint32_t {
  REASON_VN_STALE   = 1u<<0,  // VN300 갱신 지연
  REASON_RC_STALE   = 1u<<1,  // SBUS  갱신 지연
  REASON_INPUT_SPIKE= 1u<<2,  // 입력(YPR/PQR) 급변
  REASON_OUTPUT_SLEW= 1u<<3   // 출력(m) 급변
};
static inline void anom_log(uint32_t reason,
                            double vn_age_ms, double rc_age_ms,
                            uint32_t vn_seq, uint32_t rc_seq,
                            double roll, double pitch, double yaw,
                            double p, double q, double r,
                            const double m[6]) {
  anom_open_once();
  if (!g_anom) return;
  const double t_ms = ns_to_ms(now_ns());
  std::fprintf(g_anom,
    "%.3f,%u,%.3f,%.3f,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    t_ms, reason, vn_age_ms, rc_age_ms, vn_seq, rc_seq,
    roll, pitch, yaw, p, q, r,
    m[0],m[1],m[2],m[3],m[4],m[5]);
  std::fflush(g_anom);
}

// ====== Device Enable Flags ======
#define ENABLE_VN300   1
#define ENABLE_SBUS    1
#define ENABLE_BMS     0
#define ENABLE_LIHAI   0
#define ENABLE_KETI    0

// ====== Global resources ======
static std::atomic<bool> g_run{true};

static int   g_lihai_fd  = -1;
static int   g_can_fd    = -1;
static int   g_sbus_fd   = -1;
static char  g_vn_port[128]    = "/dev/ttyUSB0";
static char  g_sbus_port[128]  = "/dev/ttyACM0";
static char  g_can_if[32]      = "can0";
static char  g_lihai_port[128] = "/dev/ttyUSB2";
static std::string g_keti_host = "0.0.0.0";
static uint16_t g_keti_port    = 9999;

// ====== Global thread handles ======
static pthread_t vn_thread{};
static pthread_t sbus_thread{};
static pthread_t bms_thread{};
static pthread_t lihai_tx_thread{};
static pthread_t lihai_rx_thread{};
static pthread_t keti_thread{};
static pthread_t control_thread{};

#ifndef NUMST
#define NUMST 3 // sample time count
#endif

// Simulink config
static double simTime = 0.0;
static const double stepSize = 0.005; // 200Hz

// Simulink external mode arguments
const char_T* extArgs[] = {
    "GroundTest",
    "-w",
    "-tf", "inf",
    "-hostname", "0.0.0.0",
    "-port", "17725"
};
int extArgc = sizeof(extArgs) / sizeof(extArgs[0]);

// VN-300 object
using namespace VN;
static Sensor g_sensor;
static Registers::System::BinaryOutput1 g_binOut1;

// LiHai state error buffer
// ❗ static 제거: fcc.h의 extern State_Error_t g_state_err; 과 매칭
State_Error_t g_state_err{};

// Utility
static inline uint16_t clamp_pwm_us(double v) {
    if (!std::isfinite(v)) return 1000;
    if (v < 1000.0) v = 1000.0;
    else if (v > 2000.0) v = 2000.0;
    return static_cast<uint16_t>(std::lround(v));
}

// ================= Threads =================

// KETI server receive thread
static void* keti_thread_fn(void*) {
    keti::Server s; std::string err;
    if (!s.open(g_keti_host, g_keti_port, &err)) {
        std::fprintf(stderr, "[KETI] open fail: %s\n", err.c_str());
        return nullptr;
    }
    std::printf("[KETI] listening %s:%u\n", g_keti_host.c_str(), g_keti_port);

    while (g_run.load()) {
        auto pkt = s.recv_once(50, &err); // timeout=50ms
        if (!pkt.has_value()) continue;

        keti::Parsed p;
        if (!keti::parse_frame(pkt->bytes, p, &err)) continue;

        if (p.is_heartbeat) {
            std::printf("HB=%u from %s:%u\n",
                        (unsigned)p.heartbeat, pkt->peer_ip.c_str(), pkt->peer_port);
        } else {
            std::printf("OBJ=%u from %s:%u\n",
                        (unsigned)p.count, pkt->peer_ip.c_str(), pkt->peer_port);
        }
    }
    return nullptr;
}

// VN-300 RX thread (400Hz)
static void* vn300_thread_fn(void*) {
    while (g_run.load()) {
        { 
        std::lock_guard<std::mutex> lk(g_in_mtx);
        processSensorData(g_fcc, g_sensor, g_binOut1);
        g_vn_last.store(now_ns(), std::memory_order_relaxed);
        g_vn_seq.fetch_add(1, std::memory_order_relaxed);
        }
        usleep(2500); // 400 Hz
    }
    return nullptr;
}
static inline double sbus2047_to_us(int v2047) {
    int v = v2047;
    if (v < 172)  v = 172;
    if (v > 1810) v = 1810;
    return 1000.0 + (double)(v - 172) * (1000.0 / (1810.0 - 172.0));
}
// RC handling thread
// ⓘ 필요 헤더: <unistd.h> (이미 포함돼 있으면 생략)
static void* sbus_thread_fn(void*) {
    // 텍스트 링버퍼로 '\n' 단위 라인 만들기
    static char rb[4096];
    size_t head = 0, tail = 0;                   // 유효 구간: [tail, head)
    auto RB_SZ = sizeof(rb);

    auto rb_count = [&](){ return head - tail; };
    auto rb_push  = [&](char c){ rb[head++ % RB_SZ] = c; };
    auto rb_peek  = [&](size_t i){ return rb[(tail + i) % RB_SZ]; };
    auto rb_popn  = [&](size_t n){ tail += n; };

    char line[600];
    auto sbus2047_to_us = [](int v)->double {
        if (v < 172)  v = 172;
        if (v > 1810) v = 1810;
        return 1000.0 + (double)(v - 172) * (1000.0 / (1810 - 172));
    };

    while (g_run.load()) {
        // 1) 비블로킹 read
        char tmp[512];
        ssize_t n = ::read(g_sbus_fd, tmp, sizeof(tmp));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) rb_push(tmp[i]);
        }

        // 2) '\n'까지 한 줄 추출해서 파싱
        while (rb_count() > 0) {
            // 개행 검색
            size_t i = 0; bool found = false;
            size_t cnt = rb_count();
            for (; i < cnt; ++i) {
                if (rb_peek(i) == '\n') { found = true; break; }
            }
            if (!found) break;                 // 줄 미완 → 다음 read까지 대기

            // 라인 복사
            size_t L = std::min(i + 1, sizeof(line) - 1);
            for (size_t k = 0; k < L; ++k) line[k] = rb_peek(k);
            line[L] = '\0';
            rb_popn(i + 1);

            // 3) "SBUS Frame: " 라인만 처리
            if (strncmp(line, "SBUS Frame:", 11) == 0) {
                uint8_t frame[SBUS_FRAME_SIZE];
                int got = hex2bin(line + 12, frame, SBUS_FRAME_SIZE);  // SBUS.h 제공
                if (got == SBUS_FRAME_SIZE) {
                    std::lock_guard<std::mutex> lk(g_in_mtx);
                    parse_sbus_frame(frame, &g_fcc);                   // SBUS.h 제공
                    g_sbus_last.store(now_ns(), std::memory_order_relaxed);
                    g_sbus_seq.fetch_add(1, std::memory_order_relaxed);
                }
                // 3) "SBUS Frame: " 라인이 아니면, 공백 구분 숫자 라인인지 시도
       
            }
            else {
                // 형식: "ch0<TAB>ch1<TAB> ... <TAB>ch15<TAB>lost<TAB>failsafe\n"
                int vals[18] = {0};
                int n = 0;

                char* save = nullptr;
                for (char* tok = strtok_r(line, " \t\r\n", &save);
                    tok && n < 18;
                    tok = strtok_r(nullptr, " \t\r\n", &save))
                {
                    vals[n++] = atoi(tok);
                }

                if (n == 8) {
                    double rc_us[6];
                    for (int i = 0; i < 6; ++i)
                        rc_us[i] = sbus2047_to_us(vals[i]);

                    int lost = vals[6];
                    int failsafe = vals[7];

                    {
                        std::lock_guard<std::mutex> lk(g_in_mtx);
                        // 5채널만 쓸 경우엔 i < 5까지만 복사
                        for (int i = 0; i < 6; ++i)
                            g_fcc.rc[i] = rc_us[i];

                        g_fcc.rc_link = (failsafe ? 0 : 1);
                    }

                    g_sbus_last.store(now_ns(), std::memory_order_relaxed);
                    g_sbus_seq.fetch_add(1, std::memory_order_relaxed);
                }

            }

        }

        // 바쁘게 돌지 않게 약간 쉼
        usleep(1000); // 1 ms
    }
    return nullptr;
}



// BMS RX thread (20Hz)
static void* bms_thread_fn(void*) {
    while (g_run.load()) {
        UpdateBMSData_test(g_can_fd, &g_fcc);
        usleep(50 * 1000); // 20 Hz
    }
    return nullptr;
}

// LiHai TX thread (100Hz)
static void* lihai_tx_thread_fn(void*) {
    while (g_run.load()) {
        int txrc = send_all_packets_serial(g_lihai_fd, &g_fcc);
        if (txrc != 0) std::fprintf(stderr, "[LiHai] send error=%d\n", txrc);
        usleep(10 * 1000); // 100 Hz
    }
    return nullptr;
}

// LiHai RX thread (polling ~100Hz)
static void* lihai_rx_thread_fn(void*) {
    while (g_run.load()) {
        if (read_state_errors_packet20(g_lihai_fd, g_state_err, 20)) {
            // ������ 직접 출력은 제거(중복 방지). FCC_Data_Print에서 한 줄로 출력됨.
            // FCC_Errors_Print(&g_state_err);
        }
        usleep(10 * 1000);
    }
    return nullptr;
}

//  Simulink Control Thread
//  Simulink Control Thread 
static void* control_thread_fn(void*) {
    // External Mode init
    rtParseArgsForExtMode(extArgc, extArgs);

    printf("[EXT] Initializing model...\n");
    UAM_Flight_control_initialize();

    rtSetTFinalForExtMode(&rtmGetTFinal(UAM_Flight_control_M));
    rtExtModeCheckInit(NUMST);

    // -Non-blocking wait for start
    printf("[EXT] Waiting for start packet (Ctrl+C to abort)...\n");
    boolean_T stopReq = false;
    
    while (g_run.load() && !stopReq) {
        rtExtModeWaitForStartPkt(UAM_Flight_control_M->extModeInfo,
                                rtmGetNumSampleTimes(UAM_Flight_control_M),
                                &stopReq);

        if (!stopReq && !rtmGetStopRequested(UAM_Flight_control_M)) {
            // start packet recive
            break;
        }

        usleep(10000); // 10ms 
    }

    if (stopReq || !g_run.load()) {
        rtmSetStopRequested(UAM_Flight_control_M, true);
        printf("[EXT] terminated before start\n");
        UAM_Flight_control_terminate();
        rtExtModeShutdown(NUMST);
        return nullptr;
    }

    printf("[EXT] Connected! Sending start message...\n");
    rtERTExtModeStartMsg();


    // --- Control loop (200 Hz) ---
    while (g_run.load() && !rtmGetStopRequested(UAM_Flight_control_M)) {
        boolean_T rtmStopReq = false;
        rtExtModeOneStep(UAM_Flight_control_M->extModeInfo,
                         rtmGetNumSampleTimes(UAM_Flight_control_M),
                         &rtmStopReq);

        if (rtmStopReq) {
            rtmSetStopRequested(UAM_Flight_control_M, true);
            break;
        }

        // (1) Input mapping
        // (1) Input mapping — 입력락을 '딱 한 번'만 잡아서 일관 스냅샷 확보 + 가드/LPF
        // 1) 상태 보관(hold-last) & LPF 상태
        struct LastGood {
            double roll=0, pitch=0, yaw=0, alt=0;
            double p=0, q=0, r=0;
            double ax=0, ay=0, az=0, vn=0, ve=0, vd=0;
        } static last;

        struct LpfState {
            bool init=false;
            double roll=0, pitch=0, yaw=0, alt=0;
            double p=0, q=0, r=0;
        } static lpf;

        double alpha = 0.25; // 입력 저역통과 강도(0.2~0.3 권장)

        // 2) g_fcc에서 읽어와서 즉시 가드(락 구간 짧게!)
        double rll, ptc, yaw, alt;
        double p, q, r;
        {
            std::lock_guard<std::mutex> lk(g_in_mtx);

            // g_fcc 원본에서 꺼내기
            rll = g_fcc.roll;  ptc = g_fcc.pitch;  yaw = g_fcc.yaw;  alt = g_fcc.altitude;
            p   = (double)g_fcc.roll_rate;  q = (double)g_fcc.pitch_rate;  r = (double)g_fcc.yaw_rate;

            // 유효성 가드(필요하면 범위 조정: 라디안/라디안/라디안/미터 기준)
            if (!finite_d(rll)) rll = last.roll;  rll = clamp_d(rll, -M_PI, M_PI);
            if (!finite_d(ptc)) ptc = last.pitch; ptc = clamp_d(ptc, -M_PI, M_PI);
            if (!finite_d(yaw)) yaw = last.yaw;   yaw = unwrap_angle(last.yaw, yaw); // 래핑 해제
            if (!finite_d(alt)) alt = last.alt;   alt = clamp_d(alt, -1000.0, 10000.0);

            if (!finite_d(p)) p = last.p; p = clamp_d(p, -10.0, 10.0);   // ±10 rad/s 예시
            if (!finite_d(q)) q = last.q; q = clamp_d(q, -10.0, 10.0);
            if (!finite_d(r)) r = last.r; r = clamp_d(r, -10.0, 10.0);

            // LPF (초기 1프레임은 그대로 채움)
            if (!lpf.init) {
                lpf.init = true;
                lpf.roll=rll; lpf.pitch=ptc; lpf.yaw=yaw; lpf.alt=alt;
                lpf.p=p; lpf.q=q; lpf.r=r;
            } else {
                lpf.roll  += alpha*(rll - lpf.roll);
                lpf.pitch += alpha*(ptc - lpf.pitch);
                lpf.yaw   += alpha*(yaw - lpf.yaw);
                lpf.alt   += alpha*(alt - lpf.alt);
                lpf.p     += alpha*(p - lpf.p);
                lpf.q     += alpha*(q - lpf.q);
                lpf.r     += alpha*(r - lpf.r);
            }
            static uint64_t t_prev=0;
            uint64_t t_now = now_ns();
            if (t_now - t_prev > 1000000000ull) { // 1초
                t_prev = t_now;
                fprintf(stderr,"[CTRL] rc(us)=%.0f %.0f %.0f %.0f %.0f link=%u\n",
                    g_fcc.rc[0],g_fcc.rc[1],g_fcc.rc[2],g_fcc.rc[3],g_fcc.rc[4], g_fcc.rc_link);
            }
            // (선택) 필터/가드된 값을 g_fcc에 되적용해서 fcc_to_bus가 안전값을 보도록 함
            g_fcc.roll = lpf.roll;  g_fcc.pitch = lpf.pitch;  g_fcc.yaw = lpf.yaw;  g_fcc.altitude = lpf.alt;
            g_fcc.roll_rate = (float)lpf.p; g_fcc.pitch_rate = (float)lpf.q; g_fcc.yaw_rate = (float)lpf.r;

            // 이 시점에 Simulink 입력 구조체를 채움
            fcc_to_bus(&g_fcc, &UAM_Flight_control_U);

            // 라스트굿 갱신
            last.roll = lpf.roll; last.pitch = lpf.pitch; last.yaw = lpf.yaw; last.alt = lpf.alt;
            last.p = lpf.p; last.q = lpf.q; last.r = lpf.r;
        }
        // ※ 루프당 printf/FCC_Data_Print는 지터를 만들므로 필요할 때만 N주기 1회로 제한

        // printf("[IN ] roll=%.2f pitch=%.2f yaw=%.2f alt=%.2f\n",
        //    in_roll, in_pitch, in_yaw, in_alt);
        // ===== (A) 입력 스냅샷/나이(스테일) =====
        uint64_t t0_ns   = now_ns();
        uint64_t vn_last = g_vn_last.load(std::memory_order_relaxed);
        uint64_t rc_last = g_sbus_last.load(std::memory_order_relaxed);
        double   vn_age_ms = ns_to_ms(t0_ns - vn_last);
        double   rc_age_ms = ns_to_ms(t0_ns - rc_last);
        uint32_t vn_seq0 = g_vn_seq.load(std::memory_order_relaxed);
        uint32_t rc_seq0 = g_sbus_seq.load(std::memory_order_relaxed);

        // (입력락 블록 안에서 필터/가드 후 g_fcc에 되적용되어 있으니 그대로 읽음)
        double in_roll  = g_fcc.roll;
        double in_pitch = g_fcc.pitch;
        double in_yaw   = g_fcc.yaw;
        double in_p     = (double)g_fcc.roll_rate;
        double in_q     = (double)g_fcc.pitch_rate;
        double in_r     = (double)g_fcc.yaw_rate;

        // ===== (B) 입력 스파이크 규칙 (한 제어주기 변화 허용치) =====
        const double MAX_DANG  = 0.20;  // rad/step
        const double MAX_DRATE = 3.00;  // rad/s/step
        static bool   have_prev_in=false;
        static double prev_roll=0, prev_pitch=0, prev_yaw=0, prev_p=0, prev_q=0, prev_r=0;

        bool input_spike=false;
        if (have_prev_in) {
        if (fabs(in_roll  - prev_roll ) > MAX_DANG  ||
            fabs(in_pitch - prev_pitch) > MAX_DANG  ||
            fabs(in_yaw   - prev_yaw  ) > MAX_DANG  ||
            fabs(in_p     - prev_p    ) > MAX_DRATE ||
            fabs(in_q     - prev_q    ) > MAX_DRATE ||
            fabs(in_r     - prev_r    ) > MAX_DRATE) {
            input_spike = true;
        }
        }
        prev_roll = in_roll; prev_pitch = in_pitch; prev_yaw = in_yaw;
        prev_p = in_p; prev_q = in_q; prev_r = in_r; have_prev_in = true;

        // (2) Simulink steps
        UAM_Flight_control_step0();
        if (UAM_Flight_control_M->Timing.RateInteraction.TID0_1)
            UAM_Flight_control_step1();

        // (3) Output mapping — g_fcc.m[] 업데이트만 잠깐 보호하고 로컬 스냅샷
        double m_local[6];
        {
            std::lock_guard<std::mutex> lk(g_out_mtx);
            bus_to_fcc(&UAM_Flight_control_Y, &g_fcc);
            for (int i=0; i<6; ++i) m_local[i] = g_fcc.m[i];
        }

        const double MAX_DM_US = 120.0; // 한 스텝 허용 변화량(검증용)
        static bool have_prev_m=false;
        static double prev_m_chk[6] = {1500,1500,1500,1500,1500,1500};
        bool output_slew=false;
        if (have_prev_m) {
            for (int i=0;i<6;i++){
                if (fabs(m_local[i] - prev_m_chk[i]) > MAX_DM_US) {
                    output_slew = true;
                    break;
                }
            }
        }
        for (int i=0;i<6;i++) prev_m_chk[i] = m_local[i];
        have_prev_m = true;

        // ===== (E) 스테일 플래그 =====
        bool vn_stale = (vn_age_ms > 40.0);
        bool rc_stale = (rc_age_ms > 60.0);

        // ===== (F) 이상 시 한 줄 로깅 =====
        uint32_t reason = 0;
        if (vn_stale)    reason |= REASON_VN_STALE;
        if (rc_stale)    reason |= REASON_RC_STALE;
        if (output_slew) reason |= REASON_OUTPUT_SLEW;

        if (reason) {
            anom_log(reason, vn_age_ms, rc_age_ms,
            vn_seq0, rc_seq0,
                    g_fcc.roll, g_fcc.pitch, g_fcc.yaw,
                    g_fcc.roll_rate, g_fcc.pitch_rate, g_fcc.yaw_rate,
                    m_local);
        }

        // === (추가) 출력 안전망: 슬루 제한 + NaN/클램프 ===
        static double m_prev[6] = {1500,1500,1500,1500,1500,1500};
        const double SLEW_US_PER_STEP = 40.0; // 한 주기당 최대 변화량(예: 200Hz에서 ±40us/step)
        for (int i=0; i<6; ++i) {
            // NaN guard
            if (!std::isfinite(m_local[i])) m_local[i] = 1000.0;
            // clamp 1000~2000us
            m_local[i] = clamp_d(m_local[i], 1000.0, 2000.0);
            // slew rate limit
            double d = m_local[i] - m_prev[i];
            if (d >  SLEW_US_PER_STEP) d =  SLEW_US_PER_STEP;
            if (d < -SLEW_US_PER_STEP) d = -SLEW_US_PER_STEP;
            m_local[i] = m_prev[i] + d;
            m_prev[i]  = m_local[i];
        }

// (4) PWM output — 락 없이 로컬 스냅샷으로 출력(지터↓)
for (int i=0; i<6; ++i)
    pwm_out(i, clamp_us_u16(m_local[i]));


        // (5) Debug
        // FCC_Data_Print(&g_fcc);

        simTime += stepSize;
        usleep(5 * 1000); // 200 Hz
    }

    // --- Termination ---
    UAM_Flight_control_terminate();
    rtExtModeShutdown(NUMST);
    std::puts("[Exit] control thread terminated.");
    if (g_anom) { fclose(g_anom); g_anom = nullptr; }

    return nullptr;
}


static void on_sig(int) {
    g_run = false;

    // Simulink external mode 소켓 닫기
    rtExtModeShutdown(NUMST);

    // 디바이스 포트 닫기
    if (g_sbus_fd >= 0) { close(g_sbus_fd); g_sbus_fd = -1; }
    if (g_can_fd  >= 0) { close(g_can_fd);  g_can_fd  = -1; }
    if (g_lihai_fd>= 0) { close(g_lihai_fd);g_lihai_fd= -1; }

    pwm_all_disable();
    std::puts("[Exit] signal caught, ports closed.");
}

// ================= main =================
int main(int argc, char* argv[]) {
    // 표준출력 버퍼링 정책(멀티스레드에서도 줄 단위로 바로 보이게)
    setvbuf(stdout, nullptr, _IOLBF, 0);

    // Parse CLI arguments
    if (argc > 1) std::snprintf(g_vn_port,   sizeof(g_vn_port),   "%s", argv[1]);
    if (argc > 2) std::snprintf(g_sbus_port, sizeof(g_sbus_port), "%s", argv[2]);
    if (argc > 3) std::snprintf(g_can_if,    sizeof(g_can_if),    "%s", argv[3]);
    if (argc > 4) std::snprintf(g_lihai_port,sizeof(g_lihai_port),"%s", argv[4]);
    if (argc > 5) g_keti_port = static_cast<uint16_t>(std::stoi(argv[5]));

    // PWM init
    if (pwm_all_init() < 0) {
        std::fprintf(stderr, "[PWM] init failed\n");
        return -1;
    }

    // === Start sensor/IO threads ===
    #if ENABLE_VN300
        if (init_vn300(g_vn_port, g_sensor, g_binOut1) != 0) {
            std::fprintf(stderr, "[VN300] init error\n");
            return -2;
        }
        std::puts("[VN300] ready.");
        pthread_create(&vn_thread, nullptr, vn300_thread_fn, nullptr);
    #endif

    #if ENABLE_SBUS
        g_sbus_fd = init_serial(g_sbus_port, 115200); // SBUS.h 안의 init_serial 사용
        if (g_sbus_fd < 0) { std::fprintf(stderr, "[SBUS] open fail\n"); return -5; }
        
        int fl = fcntl(g_sbus_fd, F_GETFL, 0);
        fcntl(g_sbus_fd, F_SETFL, fl | O_NONBLOCK);

        termios tio{};
        tcgetattr(g_sbus_fd, &tio);
        cfmakeraw(&tio);             // ICANON/ECHO/POST 등 비활성
        tio.c_cc[VMIN]  = 0;         // read 비블로킹
        tio.c_cc[VTIME] = 0;         // 타임아웃 없음
        // 소프트웨어 플로우 제어 끔
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        // (필요시) 보드레이트는 init_serial에서 이미 115200 세팅
        tcsetattr(g_sbus_fd, TCSANOW, &tio);

        std::printf("[SBUS] %s opened (RAW/NONBLOCK, ASCII line input).\n", g_sbus_port);
        pthread_create(&sbus_thread, nullptr, sbus_thread_fn, nullptr);
    #endif

    #if ENABLE_BMS
        g_can_fd = open_can(std::string(g_can_if));
        if (g_can_fd < 0) {
            std::fprintf(stderr, "[CAN] open fail\n");
            return -4;
        }
        int flags = fcntl(g_can_fd, F_GETFL, 0);
        fcntl(g_can_fd, F_SETFL, flags | O_NONBLOCK);
        std::printf("[CAN] %s opened.\n", g_can_if);
        pthread_create(&bms_thread, nullptr, bms_thread_fn, nullptr);
    #endif

    #if ENABLE_LIHAI
        g_lihai_fd = lihai_open_serial_115200(g_lihai_port);
        if (g_lihai_fd < 0) {
            std::fprintf(stderr, "[LiHai] open fail\n");
            return -3;
        }
        std::printf("[LiHai] %s opened.\n", g_lihai_port);
        pthread_create(&lihai_tx_thread, nullptr, lihai_tx_thread_fn, nullptr);
        pthread_create(&lihai_rx_thread, nullptr, lihai_rx_thread_fn, nullptr);
    #endif

    #if ENABLE_KETI
        pthread_create(&keti_thread, nullptr, keti_thread_fn, nullptr);
    #endif

    // === Start control thread ===
    pthread_create(&control_thread, nullptr, control_thread_fn, nullptr);

    // === Wait for threads ===
    pthread_join(control_thread, nullptr);
    g_run = false;

    #if ENABLE_VN300
        pthread_join(vn_thread, nullptr);
        g_sensor.disconnect();
    #endif
    #if ENABLE_SBUS
        pthread_join(sbus_thread, nullptr);
        if (g_sbus_fd >= 0) close(g_sbus_fd);
    #endif
    #if ENABLE_BMS
        pthread_join(bms_thread, nullptr);
        if (g_can_fd >= 0) close(g_can_fd);
    #endif
    #if ENABLE_LIHAI
        pthread_join(lihai_tx_thread, nullptr);
        pthread_join(lihai_rx_thread, nullptr);
        if (g_lihai_fd >= 0) close(g_lihai_fd);
    #endif
    #if ENABLE_KETI
        pthread_join(keti_thread, nullptr);
    #endif

    pwm_all_disable();
    std::puts("[Exit] devices closed, process terminated.");
    return 0;
}
