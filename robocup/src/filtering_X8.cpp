#include <Arduino.h>
#include <math.h>

const int GRID_SIZE = 8;
const int NEAR_DELTA_MM = 40;          // how much closer than THIS pixel's own background counts as "object"
const int MIN_CLUSTER_SIZE = 2;        // absolute floor, still checked before the distance-aware window
const int FLATNESS_THRESHOLD_MM = 40;  // max depth spread allowed within one cluster
const float WEIGHT_DIAMETER_MM = 50.0f;
const float HFOV_DEG = 60.0f;          // SEN0628 horizontal FOV

uint16_t backgroundMap[GRID_SIZE][GRID_SIZE];
bool backgroundCalibrated = false;

struct WeightResult {
  bool found;
  float avgCol;
  float avgRow;
  int   avgDistMM;
  int   clusterSize;
};

// --- Step 1: capture the empty-scene reference at the real mount height/angle ---
// Call this once at setup(), with the sensor pointed at empty floor/wall exactly
// as it'll sit on the robot. Re-calibrate any time height or tilt angle changes.
void calibrateBackground(uint16_t distances[GRID_SIZE * GRID_SIZE]) {
  for (int r = 0; r < GRID_SIZE; r++)
    for (int c = 0; c < GRID_SIZE; c++)
      backgroundMap[r][c] = distances[r * GRID_SIZE + c];
  backgroundCalibrated = true;
}

// Optional: average a few frames first to smooth out ToF jitter before storing.
void calibrateBackgroundAveraged(uint16_t frames[][GRID_SIZE * GRID_SIZE], int numFrames) {
  long sums[GRID_SIZE][GRID_SIZE] = {0};
  for (int f = 0; f < numFrames; f++)
    for (int r = 0; r < GRID_SIZE; r++)
      for (int c = 0; c < GRID_SIZE; c++)
        sums[r][c] += frames[f][r * GRID_SIZE + c];

  for (int r = 0; r < GRID_SIZE; r++)
    for (int c = 0; c < GRID_SIZE; c++)
      backgroundMap[r][c] = sums[r][c] / numFrames;
  backgroundCalibrated = true;
}

// --- Step 2: per-pixel foreground test against that pixel's own background, not a frame-wide median ---
static bool isForeground(int row, int col, uint16_t measured) {
  if (measured == 0 || measured == 4000) return false;         // no valid return, nothing there

  uint16_t bg = backgroundMap[row][col];
  if (bg == 0 || bg == 4000) {
    // background had no return (out of range = effectively "far away") -
    // any valid measurement here is closer than that, so it's foreground
    return true;
  }

  int16_t diff = (int16_t)bg - (int16_t)measured;
  return diff > NEAR_DELTA_MM;
}

// --- Step 4: expected pixel footprint of a 50mm weight at a given distance ---
float expectedWidthPx(float distMm) {
  float halfPixelAngleRad = (HFOV_DEG / (2.0f * GRID_SIZE)) * DEG_TO_RAD;
  float footprintMmPerPx = 2.0f * distMm * tanf(halfPixelAngleRad);
  if (footprintMmPerPx < 1.0f) footprintMmPerPx = 1.0f;
  return WEIGHT_DIAMETER_MM / footprintMmPerPx;
}

bool sizeIsPlausible(int clusterSize, float distMm) {
  float expected = expectedWidthPx(distMm);
  float minSize = max(expected * 0.5f, (float)MIN_CLUSTER_SIZE);
  float maxSize = max(expected * expected * 4.0f, 6.0f); // generous area bound; floor of 6 so close-range small footprints aren't over-rejected
  return (clusterSize >= minSize) && (clusterSize <= maxSize);
}

// --- Step 5: reject flat planes (walls/corners) that touch multiple frame edges and span wide ---
bool looksLikeAPlane(int minRow, int maxRow, int minCol, int maxCol) {
  bool touchesEdge = (minRow == 0 || maxRow == GRID_SIZE - 1 || minCol == 0 || maxCol == GRID_SIZE - 1);
  int rowSpan = maxRow - minRow + 1;
  int colSpan = maxCol - minCol + 1;
  return touchesEdge && (rowSpan > 4 || colSpan > 4);
}

// --- Step 6: reject clusters with too much depth spread to be one object face ---
bool isFlatEnough(uint16_t minDist, uint16_t maxDist) {
  return (maxDist - minDist) <= FLATNESS_THRESHOLD_MM;
}

WeightResult detectWeight(uint16_t distances[GRID_SIZE * GRID_SIZE], bool verbose) {
  WeightResult result = {false, 0, 0, 0, 0};

  if (!backgroundCalibrated) {
    Serial.println("WARNING: background not calibrated - call calibrateBackground() first");
    return result;
  }

  bool visited[GRID_SIZE][GRID_SIZE] = {false};
  bool isNear[GRID_SIZE][GRID_SIZE];

  for (int r = 0; r < GRID_SIZE; r++)
    for (int c = 0; c < GRID_SIZE; c++)
      isNear[r][c] = isForeground(r, c, distances[r * GRID_SIZE + c]);

  int bestSize = 0;
  float bestSumRow = 0, bestSumCol = 0;
  int bestSumDist = 0;
  int bestMinRow = 0, bestMaxRow = 0, bestMinCol = 0, bestMaxCol = 0;
  uint16_t bestMinDist = 0, bestMaxDist = 0;

  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      if (!isNear[r][c] || visited[r][c]) continue;

      int stackR[GRID_SIZE*GRID_SIZE], stackC[GRID_SIZE*GRID_SIZE];
      int sp = 0;
      stackR[sp] = r; stackC[sp] = c; sp++;
      visited[r][c] = true;

      int size = 0; float sumRow = 0, sumCol = 0; int sumDist = 0;
      int minRow = r, maxRow = r, minCol = c, maxCol = c;
      uint16_t minDist = 65535, maxDist = 0;

      while (sp > 0) {
        sp--;
        int cr = stackR[sp], cc = stackC[sp];
        size++;
        sumRow += cr; sumCol += cc;
        uint16_t d = distances[cr * GRID_SIZE + cc];
        sumDist += d;
        if (cr < minRow) minRow = cr;
        if (cr > maxRow) maxRow = cr;
        if (cc < minCol) minCol = cc;
        if (cc > maxCol) maxCol = cc;
        if (d < minDist) minDist = d;
        if (d > maxDist) maxDist = d;

int dr[] = {-1,1,0,0}, dc[] = {0,0,-1,1};
        for (int k = 0; k < 4; k++) {
          int nr = cr + dr[k], nc = cc + dc[k];
          if (nr>=0 && nr<GRID_SIZE && nc>=0 && nc<GRID_SIZE && isNear[nr][nc] && !visited[nr][nc]) {
            uint16_t neighborDist = distances[nr * GRID_SIZE + nc];
            uint16_t newMin = min(minDist, neighborDist);
            uint16_t newMax = max(maxDist, neighborDist);
            // only grow the cluster if it stays within one object's depth range -
            // stops it bleeding into an adjacent floor/wall gradient
            if ((newMax - newMin) <= FLATNESS_THRESHOLD_MM) {
              visited[nr][nc] = true;
              stackR[sp] = nr; stackC[sp] = nc; sp++;
            }
          }
        }
      }

      if (size > bestSize) {
        bestSize = size;
        bestSumRow = sumRow; bestSumCol = sumCol; bestSumDist = sumDist;
        bestMinRow = minRow; bestMaxRow = maxRow; bestMinCol = minCol; bestMaxCol = maxCol;
        bestMinDist = minDist; bestMaxDist = maxDist;
      }
    }
  }

if (bestSize >= MIN_CLUSTER_SIZE) {
    int avgDist = bestSumDist / bestSize;
    bool sizeOk  = sizeIsPlausible(bestSize, avgDist);
    bool planeOk = !looksLikeAPlane(bestMinRow, bestMaxRow, bestMinCol, bestMaxCol);
    bool flatOk  = isFlatEnough(bestMinDist, bestMaxDist);

    if (verbose) {
      Serial.print("candidate: size="); Serial.print(bestSize);
      Serial.print(" dist="); Serial.print(avgDist);
      Serial.print(" | sizeOk="); Serial.print(sizeOk);
      Serial.print(" planeOk="); Serial.print(planeOk);
      Serial.print(" flatOk="); Serial.println(flatOk);
    }

    if (!sizeOk || !planeOk || !flatOk) return result;

    result.found = true;
    result.clusterSize = bestSize;
    result.avgRow = bestSumRow / bestSize;
    result.avgCol = bestSumCol / bestSize;
    result.avgDistMM = avgDist;
  }
  return result;
}

void printResult(WeightResult r) {
  if (r.found) {
    Serial.print("WEIGHT found | col=");
    Serial.print(r.avgCol, 1);
    Serial.print(" row=");
    Serial.print(r.avgRow, 1);
    Serial.print(" dist=");
    Serial.print(r.avgDistMM);
    Serial.print("mm size=");
    Serial.println(r.clusterSize);
  } else {
    Serial.println("no weight");
  }
}