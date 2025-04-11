#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <queue>

// Occupancy map
#define MAP_WIDTH 120
#define MAP_HEIGHT 120
#define UNKNOWN 255  //unexplored- frontier
#define FREE 0
#define OCCUPIED 1
#define ROBOT 2
const int CELL_SIZE_CM = 10;

// ---------- GLOBAL FLAGS FOR STATE COMMANDS ----------
extern bool startManualFlag;
extern bool startAutoFlag;
extern bool pauseFlag;
extern bool resumeFlag;
extern bool errorFlag;

// In globals.h (or sensor.h if more appropriate)
extern uint8_t occupancyGrid[MAP_HEIGHT][MAP_WIDTH];
extern int robotX;
extern int robotY;
extern float orientation_rad;

// ---------- FUNCTION PROTOTYPES FOR MAPPING ----------
void updateCell(int gridX, int gridY, uint8_t value);
void markLineFree(int x0, int y0, int x1, int y1);

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
