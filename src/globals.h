#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <queue>

// ---------- GLOBAL FLAGS FOR STATE COMMANDS ----------
extern bool startManualFlag;
extern bool startAutoFlag;
extern bool pauseFlag;
extern bool resumeFlag;
extern bool errorFlag;

// ---------- STRUCT FOR MANUAL COMMAND QUEUE ----------
// This struct contains minimal data: 
//   - a 'command' type or name
//   - the entire raw message (JSON or plain text), 
//   - an optional timestamp for logging.
struct ManualTaskItem {
    String commandType;      // e.g. "move", "turn", "scan", etc.
    String rawMessage;       // the original payload to parse later
    unsigned long timestamp; // optional: when it arrived
};

// ---------- QUEUE FOR MANUAL COMMANDS ----------
extern std::queue<ManualTaskItem> manualTaskQueue;

#endif  // GLOBALS_H
