#ifndef MAPS_H
#define MAPS_H

#include <Arduino.h>

// Occupancy map
#define MAP_WIDTH 120
#define MAP_HEIGHT 120
//#define UNKNOWN 255  //change to 128 - Deprecated

const int CELL_SIZE_CM = 10;

// Global map stored as floating-point log-odds
extern float occupancyGrid[MAP_HEIGHT][MAP_WIDTH];

// ===================================================
// Log-Odds Configuration (used for local updates)
// ===================================================

// Log-odds value for unknown (probability 0.5)
#define L0 0.0 // Used for initialization

// Log-odds added for a "hit" (observed as occupied by sensor model)
// Tune this based on sensor false positive rate. Lower for higher FP.
// ΔL_hit   = 0.85   # raises p from 0.5 to ~1.2f  (1.2 one ping take p=0.77)
#define LOG_ODDS_HIT 1.20f

// Log-odds added for a "miss" (observed as free along ray by sensor model)
// Tune this based on sensor false negative rate. Less negative for higher FN.
// ΔL_miss  = 0.28   # lowers p from 0.5 to ~0.43
#define LOG_ODDS_MISS -0.25f // Example: adds -0.3 log-odds towards free

// Optional: Max absolute log-odds value to clamp local updates
// Lmax     = 3.5  // saturates at p=0.97
#define L_MAX_LOCAL_CLAMP 3.5f

// Strong Free for the Robot Footprint
#define L_FREE  -2.20f     // absolutely FREE

// ===================================================
// Log-Odds to Uint8 Mapping Configuration (used for export)
// ===================================================

// --- LUT parameters -------------------------------------------------
#define L_STEP            0.05f            // quantisation step (log-odds units)
#define L_CLAMP           3.50f            // must match L_MAX_LOCAL_CLAMP or _EXPORT
#define LUT_SIZE  ((int)(2*L_CLAMP / L_STEP) + 1)

// The maximum absolute log-odds value mapped to the uint8 range ends (0 and 255)
// Choose a range that covers relevant uncertainty.
#define L_MAX_EXPORT 3.5 // Can be the same as L_MAX_LOCAL_CLAMP

// Uint8 value representing the unknown state (corresponds to log-odds 0)
#define UNKNOWN_UINT8 128

// Scaling factors for log-odds <-> uint8_t conversion for export
#define LOG_ODDS_EXPORT_SCALE (255.0 / (2.0 * L_MAX_EXPORT))
#define LOG_ODDS_EXPORT_OFFSET 128.0 // Corresponds to log_odds = 0

// global – lives in .bss (≈ 141 bytes)
extern uint8_t logOddsLUT[LUT_SIZE];

//-----------Function Prototypes ----------
void probabilisticRayUpdate(float start_wx_cm, float start_wy_cm, float end_wx_cm, float end_wy_cm, float measured_distance_cm);
void updateGridCellProbabilistic(int gridX, int gridY, float log_odds_observation);
uint8_t logOddsToUint8ForExport(float log_odds);
void updateMapFree(int robotX, int robotY);
void buildLogOddsLUT();

#endif