#include "perception/weight_detection.h"

#include <Arduino.h>
#include <math.h>

#include "config/robot_config.h"

namespace perception::weight {
namespace {
uint16_t background[config::TOF8X8_POINT_COUNT] = {};
bool calibrated = false;
WeightData latest;

bool validDistance(uint16_t distance) { return distance != 0 && distance < 4000; }

bool isForeground(uint8_t row, uint8_t column, uint16_t measured) {
  if (!validDistance(measured)) return false;
  const uint16_t bg = background[row * config::TOF8X8_GRID_SIZE + column];
  if (!validDistance(bg)) return true;
  return static_cast<int32_t>(bg) - static_cast<int32_t>(measured) >
         config::WEIGHT_NEAR_DELTA_MM;
}

float expectedWidthPixels(float distanceMm) {
  const float halfPixelAngle =
      (config::TOF8X8_HORIZONTAL_FOV_DEG / (2.0f * config::TOF8X8_GRID_SIZE)) * DEG_TO_RAD;
  const float footprint = max(1.0f, 2.0f * distanceMm * tanf(halfPixelAngle));
  return config::WEIGHT_DIAMETER_MM / footprint;
}

bool sizeIsPlausible(uint8_t clusterSize, float distanceMm, float& confidence) {
  const float expected = expectedWidthPixels(distanceMm);
  const float minimum = max(expected * 0.5f, static_cast<float>(config::WEIGHT_MIN_CLUSTER_SIZE));
  const float maximum = max(expected * expected * 4.0f, 6.0f);
  if (clusterSize < minimum || clusterSize > maximum) {
    confidence = 0.0f;
    return false;
  }
  const float targetArea = max(expected * expected, 1.0f);
  confidence = constrain(1.0f - fabsf(clusterSize - targetArea) / max(maximum, targetArea),
                         0.0f, 1.0f);
  return true;
}

bool looksLikePlane(uint8_t minRow, uint8_t maxRow, uint8_t minColumn, uint8_t maxColumn) {
  const bool touchesEdge = minRow == 0 || maxRow == config::TOF8X8_GRID_SIZE - 1 ||
                           minColumn == 0 || maxColumn == config::TOF8X8_GRID_SIZE - 1;
  return touchesEdge && ((maxRow - minRow + 1) > 4 || (maxColumn - minColumn + 1) > 4);
}
}  // namespace

void calibrateBackground(const sensors::tof8x8::Frame& frame) {
  if (!frame.valid) return;
  memcpy(background, frame.distanceMm, sizeof(background));
  calibrated = true;
}

bool backgroundCalibrated() { return calibrated; }

WeightData process8x8Frame(const sensors::tof8x8::Frame& frame) {
  latest = {};
  if (!frame.valid) return latest;
  latest.timestampMs = frame.timestampMs;
  if (!calibrated) return latest;

  bool visited[config::TOF8X8_GRID_SIZE][config::TOF8X8_GRID_SIZE] = {};
  bool foreground[config::TOF8X8_GRID_SIZE][config::TOF8X8_GRID_SIZE] = {};
  for (uint8_t row = 0; row < config::TOF8X8_GRID_SIZE; ++row) {
    for (uint8_t column = 0; column < config::TOF8X8_GRID_SIZE; ++column) {
      foreground[row][column] =
          isForeground(row, column, frame.distanceMm[row * config::TOF8X8_GRID_SIZE + column]);
    }
  }

  uint8_t bestSize = 0;
  float bestRowSum = 0.0f;
  float bestColumnSum = 0.0f;
  uint32_t bestDistanceSum = 0;
  uint8_t bestMinRow = 0, bestMaxRow = 0, bestMinColumn = 0, bestMaxColumn = 0;
  uint16_t bestMinDistance = 0, bestMaxDistance = 0;

  for (uint8_t row = 0; row < config::TOF8X8_GRID_SIZE; ++row) {
    for (uint8_t column = 0; column < config::TOF8X8_GRID_SIZE; ++column) {
      if (!foreground[row][column] || visited[row][column]) continue;

      uint8_t stackRow[config::TOF8X8_POINT_COUNT];
      uint8_t stackColumn[config::TOF8X8_POINT_COUNT];
      uint8_t stackSize = 0;
      stackRow[stackSize] = row;
      stackColumn[stackSize++] = column;
      visited[row][column] = true;

      uint8_t size = 0;
      float rowSum = 0.0f, columnSum = 0.0f;
      uint32_t distanceSum = 0;
      uint8_t minRow = row, maxRow = row, minColumn = column, maxColumn = column;
      uint16_t minDistance = UINT16_MAX, maxDistance = 0;

      while (stackSize > 0) {
        --stackSize;
        const uint8_t currentRow = stackRow[stackSize];
        const uint8_t currentColumn = stackColumn[stackSize];
        const uint16_t distance =
            frame.distanceMm[currentRow * config::TOF8X8_GRID_SIZE + currentColumn];
        ++size;
        rowSum += currentRow;
        columnSum += currentColumn;
        distanceSum += distance;
        minRow = min(minRow, currentRow);
        maxRow = max(maxRow, currentRow);
        minColumn = min(minColumn, currentColumn);
        maxColumn = max(maxColumn, currentColumn);
        minDistance = min(minDistance, distance);
        maxDistance = max(maxDistance, distance);

        const int8_t rowOffset[4] = {-1, 1, 0, 0};
        const int8_t columnOffset[4] = {0, 0, -1, 1};
        for (uint8_t direction = 0; direction < 4; ++direction) {
          const int8_t nextRow = currentRow + rowOffset[direction];
          const int8_t nextColumn = currentColumn + columnOffset[direction];
          if (nextRow < 0 || nextRow >= config::TOF8X8_GRID_SIZE || nextColumn < 0 ||
              nextColumn >= config::TOF8X8_GRID_SIZE || !foreground[nextRow][nextColumn] ||
              visited[nextRow][nextColumn]) {
            continue;
          }
          const uint16_t neighbour =
              frame.distanceMm[nextRow * config::TOF8X8_GRID_SIZE + nextColumn];
          const uint16_t newMinimum = min(minDistance, neighbour);
          const uint16_t newMaximum = max(maxDistance, neighbour);
          if (newMaximum - newMinimum <= config::WEIGHT_FLATNESS_THRESHOLD_MM) {
            visited[nextRow][nextColumn] = true;
            stackRow[stackSize] = nextRow;
            stackColumn[stackSize++] = nextColumn;
          }
        }
      }

      if (size > bestSize) {
        bestSize = size;
        bestRowSum = rowSum;
        bestColumnSum = columnSum;
        bestDistanceSum = distanceSum;
        bestMinRow = minRow;
        bestMaxRow = maxRow;
        bestMinColumn = minColumn;
        bestMaxColumn = maxColumn;
        bestMinDistance = minDistance;
        bestMaxDistance = maxDistance;
      }
    }
  }

  if (bestSize < config::WEIGHT_MIN_CLUSTER_SIZE) return latest;
  const float averageDistance = static_cast<float>(bestDistanceSum) / bestSize;
  float confidence = 0.0f;
  const bool sizeOkay = sizeIsPlausible(bestSize, averageDistance, confidence);
  const bool planeOkay = !looksLikePlane(bestMinRow, bestMaxRow, bestMinColumn, bestMaxColumn);
  const bool flatOkay = bestMaxDistance - bestMinDistance <= config::WEIGHT_FLATNESS_THRESHOLD_MM;
  const bool distanceOkay = averageDistance >= config::WEIGHT_MIN_DISTANCE_MM &&
                            averageDistance <= config::WEIGHT_MAX_DISTANCE_MM;
  if (!sizeOkay || !planeOkay || !flatOkay || !distanceOkay) return latest;

  latest.found = true;
  latest.distanceMm = averageDistance;
  latest.centroidRow = bestRowSum / bestSize;
  latest.centroidColumn = bestColumnSum / bestSize;
  latest.bearingDeg = config::TOF8X8_BEARING_SIGN * (latest.centroidColumn - 3.5f) *
                      (config::TOF8X8_HORIZONTAL_FOV_DEG / config::TOF8X8_GRID_SIZE);
  latest.clusterSize = bestSize;
  latest.confidence = confidence;
  latest.timestampMs = frame.timestampMs;
  return latest;
}

bool weightCandidateDetected() { return latest.found; }
float getWeightDistance() { return latest.distanceMm; }
float getWeightBearing() { return latest.bearingDeg; }
float getWeightConfidence() { return latest.confidence; }
const WeightData& getWeightData() { return latest; }

}  // namespace perception::weight
