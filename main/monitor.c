#include "monitor.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_attr.h"
#include "esp_timer.h"

// Per-task runtime/deadline statistics tracked by the monitor.
typedef struct {
  const char *name;
  int64_t deadline_us;
  uint32_t jobs;
  uint32_t misses;
  uint32_t active_id;
  bool active;
  int64_t start_us;
  int64_t release_us;
  int64_t max_exec_us;
  int64_t worst_lateness_us;
} task_monitor_t;

static task_monitor_t g_a = {.name = "A", .deadline_us = 10000};
static task_monitor_t g_b = {.name = "B", .deadline_us = 20000};
static task_monitor_t g_agg = {.name = "AGG", .deadline_us = 20000};
static task_monitor_t g_c = {.name = "C", .deadline_us = 50000};
static task_monitor_t g_d = {.name = "D", .deadline_us = 50000};
static task_monitor_t g_s = {.name = "S", .deadline_us = 30000};

static int64_t g_t0_us = 0;
static uint32_t g_periodic_report_every_s = 0;
static uint32_t g_final_report_after_s = 0;
static int64_t g_next_periodic_report_us = 0;
static int64_t g_final_report_deadline_us = 0;
static bool g_final_report_printed = false;

#define MON_S_RELEASE_Q_MAX 32
static volatile int64_t g_s_release_q[MON_S_RELEASE_Q_MAX];
static volatile uint32_t g_s_release_q_head = 0;
static volatile uint32_t g_s_release_q_tail = 0;
static volatile uint32_t g_s_release_q_count = 0;

// Reset one task's statistics to a clean state.
static void IRAM_ATTR reset_task(task_monitor_t *t) {
  t->jobs = 0;
  t->misses = 0;
  t->active_id = 0;
  t->active = false;
  t->start_us = 0;
  t->release_us = 0;
  t->max_exec_us = 0;
  t->worst_lateness_us = 0;
}

static int64_t base_time_us(void) {
  return (g_t0_us != 0) ? g_t0_us : esp_timer_get_time();
}

static void refresh_report_timers(void) {
  const int64_t base_us = base_time_us();

  if (g_periodic_report_every_s == 0U) {
    g_next_periodic_report_us = 0;
  } else {
    g_next_periodic_report_us = base_us + ((int64_t)g_periodic_report_every_s * 1000000LL);
  }

  if (g_final_report_after_s == 0U) {
    g_final_report_deadline_us = 0;
  } else {
    g_final_report_deadline_us = base_us + ((int64_t)g_final_report_after_s * 1000000LL);
  }
}

// Mark task activation at its logical release timestamp.
static void begin_task(task_monitor_t *t, uint32_t id, int64_t release_us) {
  t->active = true;
  t->active_id = id;
  t->release_us = release_us;
  t->start_us = esp_timer_get_time();
}

// Close task activation and update execution/deadline statistics.
// lateness_us = completion_time - absolute_deadline.
static void end_task(task_monitor_t *t) {
  if (!t->active) {
    return;
  }

  const int64_t end_us = esp_timer_get_time();
  const int64_t exec_us = end_us - t->start_us;
  const int64_t deadline_us = t->release_us + t->deadline_us;
  const int64_t lateness_us = end_us - deadline_us;

  if (exec_us > t->max_exec_us) {
    t->max_exec_us = exec_us;
  }
  if (lateness_us > t->worst_lateness_us) {
    t->worst_lateness_us = lateness_us;
  }

  t->jobs++;
  if (lateness_us > 0) {
    t->misses++;
  }

  t->active = false;
}

void monitorInit(void) {
  reset_task(&g_a);
  reset_task(&g_b);
  reset_task(&g_agg);
  reset_task(&g_c);
  reset_task(&g_d);
  reset_task(&g_s);

  g_t0_us = 0;
  g_final_report_printed = false;
  g_s_release_q_head = 0;
  g_s_release_q_tail = 0;
  g_s_release_q_count = 0;
  refresh_report_timers();
}

void IRAM_ATTR synch(void) {
  // SYNC starts a new measurement epoch.
  g_t0_us = esp_timer_get_time();
  reset_task(&g_a);
  reset_task(&g_b);
  reset_task(&g_agg);
  reset_task(&g_c);
  reset_task(&g_d);
  reset_task(&g_s);

  g_final_report_printed = false;
  g_s_release_q_head = 0;
  g_s_release_q_tail = 0;
  g_s_release_q_count = 0;
  refresh_report_timers();
}

void IRAM_ATTR notifySRelease(void) {
  const int64_t now_us = esp_timer_get_time();
  if (g_s_release_q_count < MON_S_RELEASE_Q_MAX) {
    g_s_release_q[g_s_release_q_tail] = now_us;
    g_s_release_q_tail = (g_s_release_q_tail + 1U) % MON_S_RELEASE_Q_MAX;
    g_s_release_q_count++;
  }
}

void beginTaskA(uint32_t id) { begin_task(&g_a, id, g_t0_us + ((int64_t)id * 10000)); }
void endTaskA(void) { end_task(&g_a); }

void beginTaskB(uint32_t id) { begin_task(&g_b, id, g_t0_us + ((int64_t)id * 20000)); }
void endTaskB(void) { end_task(&g_b); }

void beginTaskAGG(uint32_t id) { begin_task(&g_agg, id, g_t0_us + ((int64_t)id * 20000)); }
void endTaskAGG(void) { end_task(&g_agg); }

void beginTaskC(uint32_t id) { begin_task(&g_c, id, g_t0_us + ((int64_t)id * 50000)); }
void endTaskC(void) { end_task(&g_c); }

void beginTaskD(uint32_t id) { begin_task(&g_d, id, g_t0_us + ((int64_t)id * 50000)); }
void endTaskD(void) { end_task(&g_d); }

void beginTaskS(uint32_t id) {
  int64_t release_us = esp_timer_get_time();
  if (g_s_release_q_count > 0U) {
    release_us = g_s_release_q[g_s_release_q_head];
    g_s_release_q_head = (g_s_release_q_head + 1U) % MON_S_RELEASE_Q_MAX;
    g_s_release_q_count--;
  }
  begin_task(&g_s, id, release_us);
}
void endTaskS(void) { end_task(&g_s); }

void monitorSetPeriodicReportEverySeconds(uint32_t seconds) {
  g_periodic_report_every_s = seconds;
  refresh_report_timers();
}

void monitorSetFinalReportAfterSeconds(uint32_t seconds) {
  g_final_report_after_s = seconds;
  g_final_report_printed = false;
  refresh_report_timers();
}

void monitorPrintFinalReport(void) {
  printf("FINAL_REPORT_BEGIN\n");
  monitorReport();
  printf("FINAL_REPORT_END\n");
}

// Print one task line in the same format expected by the final report.
static void report_one(const task_monitor_t *t) {
  printf("[MON] %s jobs=%" PRIu32 " misses=%" PRIu32 " max_exec=%" PRIi64
         "us worst_late=%" PRIi64 "us\n",
         t->name, t->jobs, t->misses, t->max_exec_us, t->worst_lateness_us);
}

void monitorReport(void) {
  printf("[MON] T0=%" PRIi64 " us\n", g_t0_us);
  report_one(&g_a);
  report_one(&g_b);
  report_one(&g_agg);
  report_one(&g_c);
  report_one(&g_d);
  report_one(&g_s);
}

bool monitorPollReports(void) {
  const int64_t now_us = esp_timer_get_time();

  if (g_next_periodic_report_us > 0 && g_periodic_report_every_s > 0U &&
      now_us >= g_next_periodic_report_us) {
    const int64_t period_us = (int64_t)g_periodic_report_every_s * 1000000LL;
    const int64_t missed_periods = (now_us - g_next_periodic_report_us) / period_us;
    g_next_periodic_report_us += (missed_periods + 1) * period_us;
    monitorReport();
  }

  if (!g_final_report_printed && g_final_report_deadline_us > 0 &&
      now_us >= g_final_report_deadline_us) {
    monitorPrintFinalReport();
    g_final_report_printed = true;
    return true;
  }

  return false;
}

bool monitorAllDeadlinesMet(void) {
  return (g_a.misses == 0) && (g_b.misses == 0) && (g_agg.misses == 0) &&
         (g_c.misses == 0) && (g_d.misses == 0) && (g_s.misses == 0);
}
