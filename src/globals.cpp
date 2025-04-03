#include "globals.h"

// Global flags or variables updated by MQTT callbacks:
bool startManualFlag = false;
bool startAutoFlag = false;
bool pauseFlag = false;
bool resumeFlag = false;
bool errorFlag = false;
// ... plus any manual motion commands

// Define (allocate) the queue here
std::queue<ManualTaskItem> manualTaskQueue;
