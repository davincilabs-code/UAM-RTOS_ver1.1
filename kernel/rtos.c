#define _XOPEN_SOURCE 700
#include "rtos.h"

#include <ucontext.h>
#include <signal.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>

typedef enum { TASK_UNUSED=0, TASK_READY, TASK_RUNNING, TASK_DELAYED, TASK_BLOCKED } task_state_t;

typedef struct rtos_task {
    char name[RTOS_NAME_MAX];
    ucontext_t ctx;
    uint8_t priority;
    task_state_t state;

    rtos_task_fn fn;
    void* arg;

    uint8_t* stack;
    size_t stack_size;

    uint64_t wake_tick;

    struct rtos_task* next;
} rtos_task_t;

/* Ready lists per priority (simple singly linked list) */
static rtos_task_t g_tasks[RTOS_MAX_TASKS];
static rtos_task_t* g_ready[RTOS_MAX_PRIORITY];
static rtos_task_t* g_delayed = NULL;

static rtos_task_t* g_current = NULL;
static ucontext_t g_kernel_ctx;

static volatile sig_atomic_t g_need_resched = 0;
static volatile sig_atomic_t g_in_kernel = 0;

static uint32_t g_tick_hz = 1000;
static volatile uint64_t g_tick = 0;

static void critical_enter(sigset_t* prev)
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGALRM);
    sigprocmask(SIG_BLOCK, &set, prev);
}

static void critical_exit(const sigset_t* prev)
{
    sigprocmask(SIG_SETMASK, prev, NULL);
}

static void ready_push(rtos_task_t* t)
{
    uint8_t p = t->priority;
    t->next = NULL;
    if (!g_ready[p]) {
        g_ready[p] = t;
        return;
    }
    /* append */
    rtos_task_t* cur = g_ready[p];
    while (cur->next) cur = cur->next;
    cur->next = t;
}

static rtos_task_t* ready_pop_highest(void)
{
    for (int p = RTOS_MAX_PRIORITY - 1; p >= 0; --p) {
        rtos_task_t* h = g_ready[p];
        if (h) {
            g_ready[p] = h->next;
            h->next = NULL;
            return h;
        }
    }
    return NULL;
}

static void delayed_insert_sorted(rtos_task_t* t)
{
    t->next = NULL;
    if (!g_delayed || t->wake_tick < g_delayed->wake_tick) {
        t->next = g_delayed;
        g_delayed = t;
        return;
    }
    rtos_task_t* cur = g_delayed;
    while (cur->next && cur->next->wake_tick <= t->wake_tick) {
        cur = cur->next;
    }
    t->next = cur->next;
    cur->next = t;
}

static void tick_wakeup_due_tasks(void)
{
    /* move due delayed tasks to ready */
    while (g_delayed && g_delayed->wake_tick <= g_tick) {
        rtos_task_t* t = g_delayed;
        g_delayed = t->next;
        t->next = NULL;
        t->state = TASK_READY;
        ready_push(t);
    }
}

/* NOTE: handler must remain tiny */
static void sigalrm_handler(int signo)
{
    (void)signo;
    g_tick++;
    tick_wakeup_due_tasks();
    g_need_resched = 1;
}

/* Task trampoline */
static void task_entry(uintptr_t tptr)
{
    rtos_task_t* t = (rtos_task_t*)tptr;
    t->fn(t->arg);

    /* If task returns, park it forever */
    while (1) {
        rtos_delay_ms(1000);
    }
}

/* Scheduler: pick next, context switch */
static void schedule(void)
{
    sigset_t prev;
    critical_enter(&prev);

    rtos_task_t* next = ready_pop_highest();
    if (!next) {
        /* no ready task: run current if valid, else idle by sleeping */
        critical_exit(&prev);
        usleep(1000);
        return;
    }

    rtos_task_t* prev_task = g_current;
    g_current = next;
    next->state = TASK_RUNNING;

    /* If prev task was running, make it ready (round-robin within same prio on yield) */
    if (prev_task && prev_task->state == TASK_RUNNING) {
        prev_task->state = TASK_READY;
        ready_push(prev_task);
    }

    g_need_resched = 0;
    critical_exit(&prev);

    if (prev_task) {
        swapcontext(&prev_task->ctx, &next->ctx);
    } else {
        setcontext(&next->ctx);
    }
}

rtos_status_t rtos_init(uint32_t tick_hz)
{
    if (tick_hz == 0) return RTOS_ERR;
    memset(g_tasks, 0, sizeof(g_tasks));
    memset(g_ready, 0, sizeof(g_ready));
    g_delayed = NULL;
    g_current = NULL;
    g_tick = 0;
    g_tick_hz = tick_hz;
    g_need_resched = 0;
    g_in_kernel = 0;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigalrm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGALRM, &sa, NULL) != 0) {
        return RTOS_ERR;
    }
    return RTOS_OK;
}

rtos_status_t rtos_task_create(const char* name,
                               rtos_task_fn fn,
                               void* arg,
                               size_t stack_size,
                               uint8_t priority,
                               rtos_task_handle_t* out_handle)
{
    if (!fn || !name || stack_size < 16*1024 || priority >= RTOS_MAX_PRIORITY) return RTOS_ERR;

    sigset_t prev;
    critical_enter(&prev);

    int idx = -1;
    for (int i=0;i<RTOS_MAX_TASKS;i++){
        if (g_tasks[i].state == TASK_UNUSED) { idx=i; break; }
    }
    if (idx < 0) { critical_exit(&prev); return RTOS_FULL; }

    rtos_task_t* t = &g_tasks[idx];
    memset(t, 0, sizeof(*t));
    strncpy(t->name, name, RTOS_NAME_MAX-1);
    t->priority = priority;
    t->state = TASK_READY;
    t->fn = fn;
    t->arg = arg;
    t->stack_size = stack_size;
    t->stack = (uint8_t*)malloc(stack_size);
    if (!t->stack) { memset(t,0,sizeof(*t)); t->state = TASK_UNUSED; critical_exit(&prev); return RTOS_ERR; }

    getcontext(&t->ctx);
    t->ctx.uc_stack.ss_sp = t->stack;
    t->ctx.uc_stack.ss_size = t->stack_size;
    t->ctx.uc_link = &g_kernel_ctx; /* if task exits, return to kernel context */
    makecontext(&t->ctx, (void(*)(void))task_entry, 1, (uintptr_t)t);

    ready_push(t);

    if (out_handle) *out_handle = (rtos_task_handle_t)t;

    critical_exit(&prev);
    return RTOS_OK;
}

void rtos_start(void)
{
    /* Start periodic tick */
    struct itimerval it;
    memset(&it, 0, sizeof(it));
    uint32_t usec = (uint32_t)(1000000UL / g_tick_hz);
    it.it_interval.tv_sec = 0;
    it.it_interval.tv_usec = usec;
    it.it_value = it.it_interval;
    setitimer(ITIMER_REAL, &it, NULL);

    /*
     * Capture kernel context so tasks can "return" here on rtos_delay_ms()
     * via swapcontext(&task, &g_kernel_ctx).
     */
    if (getcontext(&g_kernel_ctx) == 0) {
        /* First time: start first task */
        if (g_current == NULL) {
            schedule();
        }
    }

    /* Kernel loop: run whenever a task yields/delays back to kernel */
    while (1) {
        schedule();
    }
}

void rtos_yield(void)
{
    if (!g_current) return;

    sigset_t prev;
    critical_enter(&prev);
    /* mark need resched; schedule will round-robin current */
    g_need_resched = 1;
    critical_exit(&prev);

    schedule();
}

void rtos_delay_ms(uint32_t ms)
{
    if (!g_current) return;

    uint64_t ticks = (ms * (uint64_t)g_tick_hz + 999) / 1000;
    if (ticks == 0) ticks = 1;

    sigset_t prev;
    critical_enter(&prev);

    g_current->wake_tick = g_tick + ticks;
    g_current->state = TASK_DELAYED;
    delayed_insert_sorted(g_current);

    /* Remove from running; next schedule won't requeue it */
    rtos_task_t* prev_task = g_current;
    g_current = NULL;

    g_need_resched = 0;
    critical_exit(&prev);

    /* Switch to next task */
    swapcontext(&prev_task->ctx, &g_kernel_ctx);
    /* When resumed later, execution continues here */
}

void rtos_poll_delay_1tick(void)
{
    /* Simple polling delay for sync primitives without direct kernel blocking. */
    rtos_delay_ms((uint32_t)(1000 / g_tick_hz));
}

uint64_t rtos_tick_get(void)
{
    return g_tick;
}

uint64_t rtos_tick_to_ms(uint64_t tick)
{
    return (tick * 1000ULL) / (uint64_t)g_tick_hz;
}

void rtos_log(const char* tag, const char* fmt, ...)
{
    uint64_t t = rtos_tick_to_ms(rtos_tick_get());

    fprintf(stdout, "[%8llums] %-10s ", (unsigned long long)t, tag ? tag : "LOG");

    va_list ap;
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);

    fputc('\n', stdout);
    fflush(stdout);

    /* Co-operative preemption point */
    if (g_need_resched) {
        rtos_yield();
    }
}
