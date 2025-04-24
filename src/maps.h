#ifndef MAPS_H
#define MAPS_H

#include <Arduino.h>

// Occupancy map
#define MAP_WIDTH 120
#define MAP_HEIGHT 120
#define UNKNOWN 255  //change to 128 - Deprecated
//#define FREE 0  // depricated
//#define OCCUPIED 1 // depricated
//#define ROBOT 2 // depricated
const int CELL_SIZE_CM = 10;

//extern uint8_t occupancyGrid[MAP_HEIGHT][MAP_WIDTH];  //chagne to float

// Global map stored as floating-point log-odds
extern float occupancyGrid[MAP_HEIGHT][MAP_WIDTH];

// ===================================================
// Log-Odds Configuration (used for local updates)
// ===================================================
//other suggested values
// ΔL_hit   = 0.85   # raises p from 0.5 to ~0.7
// ΔL_miss  = 0.28   # lowers p from 0.5 to ~0.43
// decay    = 0.05   # toward zero each 0.5 s
// Lmax     = 3.5

// Log-odds value for unknown (probability 0.5)
#define L0 0.0 // Used for initialization

// Log-odds added for a "hit" (observed as occupied by sensor model)
// Tune this based on sensor false positive rate. Lower for higher FP.
#define LOG_ODDS_HIT 0.5 // Example: adds 0.5 log-odds towards occupied

// Log-odds added for a "miss" (observed as free along ray by sensor model)
// Tune this based on sensor false negative rate. Less negative for higher FN.
#define LOG_ODDS_MISS -0.3 // Example: adds -0.3 log-odds towards free

// Optional: Max absolute log-odds value to clamp local updates
// Helps prevent floating-point issues with extremely high/low confidence
#define L_MAX_LOCAL_CLAMP 8.0


// ===================================================
// Log-Odds to Uint8 Mapping Configuration (used for export)
// ===================================================

// The maximum absolute log-odds value mapped to the uint8 range ends (0 and 255)
// Choose a range that covers relevant uncertainty.
#define L_MAX_EXPORT 8.0 // Can be the same as L_MAX_LOCAL_CLAMP

// Uint8 value representing the unknown state (corresponds to log-odds 0)
#define UNKNOWN_UINT8 128

// Scaling factors for log-odds <-> uint8_t conversion for export
#define LOG_ODDS_EXPORT_SCALE (255.0 / (2.0 * L_MAX_EXPORT))
#define LOG_ODDS_EXPORT_OFFSET 128.0 // Corresponds to log_odds = 0

//-----------Function Prototypes ----------
void probabilisticRayUpdate(float start_wx_cm, float start_wy_cm, float end_wx_cm, float end_wy_cm, float measured_distance_cm);
void updateGridCellProbabilistic(int gridX, int gridY, float log_odds_observation);
uint8_t logOddsToUint8ForExport(float log_odds);

#endif