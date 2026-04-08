#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Reset all monitor counters/state before first use.
void monitorInit(void);
// Define SYNC/T0 and reset per-task statistics for a new run.
void synch(void);
// Record one sporadic release timestamp (call this as close as possible to S event edge).
void notifySRelease(void);

// Task hooks: call begin at task start and end at task completion.
// Periodic task releases are derived from SYNC/T0 and the provided id.
void beginTaskA(uint32_t id);
void endTaskA(void);

void beginTaskB(uint32_t id);
void endTaskB(void);

void beginTaskAGG(uint32_t id);
void endTaskAGG(void);

void beginTaskC(uint32_t id);
void endTaskC(void);

void beginTaskD(uint32_t id);
void endTaskD(void);

// Sporadic task release is taken from the notifySRelease queue.
void beginTaskS(uint32_t id);
void endTaskS(void);

// Time-based report control:
// 0 disables the respective report type.
void monitorSetPeriodicReportEverySeconds(uint32_t seconds);
void monitorSetFinalReportAfterSeconds(uint32_t seconds);
// Poll report timers; returns true once when final report gets printed.
bool monitorPollReports(void);
// Print current monitor summary (T0 + per-task stats).
void monitorReport(void);
// Print final report block (header + monitor summary).
void monitorPrintFinalReport(void);
// True only if all monitored tasks have zero misses so far.
bool monitorAllDeadlinesMet(void);

#ifdef __cplusplus
}
#endif
