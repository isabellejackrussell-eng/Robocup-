#include <Arduino.h>

const int GRID_SIZE = 8;
const int NEAR_THRESHOLD_MM = 80;   // how much closer than background counts as "object"
const int MIN_CLUSTER_SIZE = 2;     // ignore single noisy pixels

struct WeightResult {
  bool found;
  float avgCol;      // 0-7, column centroid (use for left/right direction)
  float avgRow;
  int   avgDistMM;
  int   clusterSize;
};

int backgroundMM(uint16_t distances[GRID_SIZE * GRID_SIZE]) {
  uint16_t sorted[GRID_SIZE * GRID_SIZE];
  memcpy(sorted, distances, sizeof(sorted));
  for (int i = 1; i < GRID_SIZE * GRID_SIZE; i++) {
    int key = sorted[i], j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
    sorted[j+1] = key;
  }
  return sorted[GRID_SIZE * GRID_SIZE / 2]; // median as a stand-in for "wall/floor"
}

WeightResult detectWeight(uint16_t distances[GRID_SIZE * GRID_SIZE]) {
  WeightResult result = {false, 0, 0, 0, 0};
  int background = backgroundMM(distances);

  bool visited[GRID_SIZE][GRID_SIZE] = {false};
  bool isNear[GRID_SIZE][GRID_SIZE];

  for (int r = 0; r < GRID_SIZE; r++)
    for (int c = 0; c < GRID_SIZE; c++) {
      int v = distances[r * GRID_SIZE + c];
      isNear[r][c] = (v > 0) && (v < background - NEAR_THRESHOLD_MM);
    }

  int bestSize = 0;
  float bestSumRow = 0, bestSumCol = 0;
  int bestSumDist = 0;

  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      if (!isNear[r][c] || visited[r][c]) continue;

      int stackR[GRID_SIZE*GRID_SIZE], stackC[GRID_SIZE*GRID_SIZE];
      int sp = 0;
      stackR[sp] = r; stackC[sp] = c; sp++;
      visited[r][c] = true;

      int size = 0; float sumRow = 0, sumCol = 0; int sumDist = 0;

      while (sp > 0) {
        sp--;
        int cr = stackR[sp], cc = stackC[sp];
        size++;
        sumRow += cr; sumCol += cc;
        sumDist += distances[cr * GRID_SIZE + cc];

        int dr[] = {-1,1,0,0}, dc[] = {0,0,-1,1};
        for (int k = 0; k < 4; k++) {
          int nr = cr + dr[k], nc = cc + dc[k];
          if (nr>=0 && nr<GRID_SIZE && nc>=0 && nc<GRID_SIZE && isNear[nr][nc] && !visited[nr][nc]) {
            visited[nr][nc] = true;
            stackR[sp] = nr; stackC[sp] = nc; sp++;
          }
        }
      }

      if (size > bestSize) {
        bestSize = size;
        bestSumRow = sumRow; bestSumCol = sumCol; bestSumDist = sumDist;
      }
    }
  }

  if (bestSize >= MIN_CLUSTER_SIZE) {
    result.found = true;
    result.clusterSize = bestSize;
    result.avgRow = bestSumRow / bestSize;
    result.avgCol = bestSumCol / bestSize;
    result.avgDistMM = bestSumDist / bestSize;
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