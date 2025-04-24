#include "maps.h"
#include "globals.h"
#include "sensor.h"

// This is the DEFINITION of the occupancyGrid variable
float occupancyGrid[MAP_HEIGHT][MAP_WIDTH];

// ==================================
void initializeMap() {
    for (int i = 0; i < MAP_HEIGHT; i++) {
      for (int j = 0; j < MAP_WIDTH; j++) {
        occupancyGrid[i][j] = L0;  //Initialize as Log-odds UNKNOWN
      }
    }
  }
  
  //DEPRECATED  by updateGridCellProbabilistic()
//   void updateCell(int gridX, int gridY, uint8_t value) {
//     if (gridX >= 0 && gridX < MAP_WIDTH && gridY >= 0 && gridY < MAP_HEIGHT) {
//       occupancyGrid[gridY][gridX] = value;
//     }
//   }
  
  //DEPRECATED
//   void markLineFree(int x0, int y0, int x1, int y1) {
//     int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
//     int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
//     int err = dx + dy, e2; // error value e_xy
  
//     while (true) {
//       updateCell(x0, y0, FREE);  // mark current cell as FREE
//       if (x0 == x1 && y0 == y1) break;
//       e2 = 2 * err;
//       if (e2 >= dy) { err += dy; x0 += sx; }
//       if (e2 <= dx) { err += dx; y0 += sy; }
//     }
//   }

// ===================================================
// Conversion Functions (only used for export)
// ===================================================

// Converts a floating-point log-odds value to a uint8_t grid value for export
uint8_t logOddsToUint8ForExport(float log_odds) {
    // Clamp log-odds to the export mapping range
    if (log_odds > L_MAX_EXPORT) log_odds = L_MAX_EXPORT;
    if (log_odds < -L_MAX_EXPORT) log_odds = -L_MAX_EXPORT;

    // Map log-odds to 0-255 range
    float mapped_value = log_odds * LOG_ODDS_EXPORT_SCALE + LOG_ODDS_EXPORT_OFFSET;

    // Clamp the result to the uint8_t range [0, 255]
    if (mapped_value < 0.0) mapped_value = 0.0;
    if (mapped_value > 255.0) mapped_value = 255.0;

    // Round and cast to uint8_t
    return (uint8_t)round(mapped_value);
}


// ===================================================
// Probabilistic Update Function for a Single Cell (uses float)
// ===================================================

// Updates a single grid cell probabilistically using float log-odds
void updateGridCellProbabilistic(int gridX, int gridY, float log_odds_observation) {
    if (gridX >= 0 && gridX < MAP_WIDTH && gridY >= 0 && gridY < MAP_HEIGHT) {
        // Get current floating-point log-odds
        float current_log_odds = occupancyGrid[gridY][gridX];

        // Calculate the new log-odds using the update rule: L_new = L_old + L_observation
        // (Since L0 is 0, we just add the observation log-odds)
        float new_log_odds = current_log_odds + log_odds_observation;

        // Optional: Clamp the new log-odds locally
        if (new_log_odds > L_MAX_LOCAL_CLAMP) new_log_odds = L_MAX_LOCAL_CLAMP;
        if (new_log_odds < -L_MAX_LOCAL_CLAMP) new_log_odds = -L_MAX_LOCAL_CLAMP;


        // Update the cell in the grid (using float)
        occupancyGrid[gridY][gridX] = new_log_odds;
    }
}

// Helper function to get grid coordinates from world coordinates (cm)
void worldToGrid(float wx_cm, float wy_cm, int* gridX, int* gridY) {
    // Assuming (0,0) world corresponds to the center of the grid.
    // Adjust this if your grid origin is different.
    *gridX = (int)round(wx_cm / CELL_SIZE_CM) + MAP_WIDTH / 2;
    *gridY = (int)round(wy_cm / CELL_SIZE_CM) + MAP_HEIGHT / 2;
}

// Performs probabilistic update along a ray using the float grid
void probabilisticRayUpdate(float start_wx_cm, float start_wy_cm, float end_wx_cm, float end_wy_cm, float measured_distance_cm) {
    // Use Bresenham-like algorithm to iterate through cells along the ray
    // from start_wx_cm, start_wy_cm towards end_wx_cm, end_wy_cm

    int x0, y0, x1, y1;
    worldToGrid(start_wx_cm, start_wy_cm, &x0, &y0);
    worldToGrid(end_wx_cm, end_wy_cm, &x1, &y1);

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; // error value e_xy

    int current_x = x0;
    int current_y = y0;

     // Use a boolean flag to know if we are currently at the endpoint cell
    bool reached_endpoint_cell = false;


    while (true) {
        // Calculate distance from ray start to the current cell's center (approx)
        float cell_center_wx_cm = (current_x - MAP_WIDTH / 2) * CELL_SIZE_CM;
        float cell_center_wy_cm = (current_y - MAP_HEIGHT / 2) * CELL_SIZE_CM;
        float dist_to_cell_center_cm = sqrt(pow(cell_center_wx_cm - start_wx_cm, 2) + pow(cell_center_wy_cm - start_wy_cm, 2));


        // Check if we've reached the intended grid cell for the measurement endpoint
        if ((current_x == x1 && current_y == y1)) {
            reached_endpoint_cell = true;
        }

        // Apply update based on whether it's the endpoint and if a valid distance was measured
        if (measured_distance_cm < SENSOR_MAX_DIST && reached_endpoint_cell) {
             // This is the cell where the sensor reported an obstacle
            updateGridCellProbabilistic(current_x, current_y, LOG_ODDS_HIT);
        } else if (dist_to_cell_center_cm < measured_distance_cm || measured_distance_cm >= SENSOR_MAX_DIST) {
             // This cell is along the free path or the sensor maxed out (ray went through)
             updateGridCellProbabilistic(current_x, current_y, LOG_ODDS_MISS);
        }
        // Note: Cells beyond the measured distance (if not max range) and not along the ray are not updated.


        if (current_x == x1 && current_y == y1 && reached_endpoint_cell) break; // Stop when we update the endpoint cell


        e2 = 2 * err;
        int prev_x = current_x; // Store previous x before potential move
        if (e2 >= dy) { err += dy; current_x += sx; }
        if (e2 <= dx) { err += dx; current_y += sy; }

        // If we moved and are now past the intended endpoint cell, stop.
        // This prevents overshooting when the endpoint is between cells.
        if ((sx > 0 && current_x > x1 && prev_x <= x1) || (sx < 0 && current_x < x1 && prev_x >= x1) ||
            (sy > 0 && current_y > y1 && y0 <= y1) || (sy < 0 && current_y < y1 && y0 >= y1)) {
             if (!reached_endpoint_cell && measured_distance_cm < SENSOR_MAX_DIST) {
                  // If we somehow overshot without marking the hit cell,
                  // make sure to mark the last valid cell along the line towards the endpoint
                  // as the hit cell. This part of the logic is tricky and depends on
                  // the specific Bresenham implementation and desired behavior.
                  // For simplicity here, we assume the cell at x1, y1 will be visited if valid.
             }
            // A more robust ray tracing algorithm might be needed for precise cell updates.
            // For this example, the loop condition `current_x == x1 && current_y == y1` should be sufficient
            // if x1, y1 calculation is accurate to cell indices.
        }
    }
}