#include "filter_weightDetect.h"

#include <math.h>

namespace filter_weightDetect {
namespace {

constexpr uint16_t kNearDeltaMm = 40;
constexpr uint8_t kMinimumClusterSize = 2;
constexpr uint16_t kFlatnessThresholdMm = 40;
constexpr float kWeightDiameterMm = 50.0f;
constexpr float kHorizontalFieldOfViewDegrees = 60.0f;
constexpr uint16_t kOutOfRangeDistanceMm = 4000;

uint16_t backgroundMap[hd_raw_tof8x8::kGridSize]
                      [hd_raw_tof8x8::kGridSize] = {};
bool backgroundCalibrated = false;

bool isValidDistance(uint16_t distanceMm) {
  return distanceMm != 0 && distanceMm != kOutOfRangeDistanceMm;
}

bool isForeground(uint8_t row, uint8_t column, uint16_t measuredMm) {
  if (!isValidDistance(measuredMm)) {
    return false;
  }

  const uint16_t backgroundMm = backgroundMap[row][column];
  if (!isValidDistance(backgroundMm)) {
    return true;
  }

  return static_cast<int32_t>(backgroundMm) - measuredMm > kNearDeltaMm;
}

float expectedWidthPixels(float distanceMm) {
  const float halfPixelAngleRadians =
      (kHorizontalFieldOfViewDegrees /
       (2.0f * hd_raw_tof8x8::kGridSize)) * DEG_TO_RAD;
  float millimetresPerPixel =
      2.0f * distanceMm * tanf(halfPixelAngleRadians);

  if (millimetresPerPixel < 1.0f) {
    millimetresPerPixel = 1.0f;
  }

  return kWeightDiameterMm / millimetresPerPixel;
}

bool sizeIsPlausible(uint8_t clusterSize, float distanceMm) {
  const float expectedWidth = expectedWidthPixels(distanceMm);
  const float minimumSize =
      max(expectedWidth * 0.5f, static_cast<float>(kMinimumClusterSize));
  const float maximumSize =
      max(expectedWidth * expectedWidth * 4.0f, 6.0f);

  return clusterSize >= minimumSize && clusterSize <= maximumSize;
}

bool looksLikeAPlane(
    uint8_t minimumRow,
    uint8_t maximumRow,
    uint8_t minimumColumn,
    uint8_t maximumColumn) {
  const bool touchesEdge =
      minimumRow == 0 ||
      maximumRow == hd_raw_tof8x8::kGridSize - 1 ||
      minimumColumn == 0 ||
      maximumColumn == hd_raw_tof8x8::kGridSize - 1;
  const uint8_t rowSpan = maximumRow - minimumRow + 1;
  const uint8_t columnSpan = maximumColumn - minimumColumn + 1;

  return touchesEdge && (rowSpan > 4 || columnSpan > 4);
}

bool isFlatEnough(uint16_t minimumDistanceMm, uint16_t maximumDistanceMm) {
  return maximumDistanceMm - minimumDistanceMm <= kFlatnessThresholdMm;
}

}  // namespace

bool calibrateBackground(const hd_raw_tof8x8::Frame& distances) {
  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    for (uint8_t column = 0; column < hd_raw_tof8x8::kGridSize; ++column) {
      backgroundMap[row][column] =
          distances[row * hd_raw_tof8x8::kGridSize + column];
    }
  }

  backgroundCalibrated = true;
  return true;
}

bool calibrateBackgroundAveraged(
    const hd_raw_tof8x8::Frame* frames,
    size_t frameCount) {
  if (frames == nullptr || frameCount == 0) {
    return false;
  }

  uint32_t sums[hd_raw_tof8x8::kPointCount] = {};
  uint16_t validCounts[hd_raw_tof8x8::kPointCount] = {};

  for (size_t frameIndex = 0; frameIndex < frameCount; ++frameIndex) {
    for (size_t point = 0; point < hd_raw_tof8x8::kPointCount; ++point) {
      const uint16_t distanceMm = frames[frameIndex][point];
      if (isValidDistance(distanceMm)) {
        sums[point] += distanceMm;
        ++validCounts[point];
      }
    }
  }

  for (size_t point = 0; point < hd_raw_tof8x8::kPointCount; ++point) {
    const uint8_t row = point / hd_raw_tof8x8::kGridSize;
    const uint8_t column = point % hd_raw_tof8x8::kGridSize;
    backgroundMap[row][column] = validCounts[point] == 0
        ? kOutOfRangeDistanceMm
        : sums[point] / validCounts[point];
  }

  backgroundCalibrated = true;
  return true;
}

bool isBackgroundCalibrated() {
  return backgroundCalibrated;
}

void clearBackgroundCalibration() {
  backgroundCalibrated = false;
}

WeightResult detectWeight(
    const hd_raw_tof8x8::Frame& distances,
    bool verbose) {
  WeightResult result = {false, 0.0f, 0.0f, 0, 0};

  if (!backgroundCalibrated) {
    if (verbose) {
      Serial.println(
          "WARNING: background not calibrated - call calibrateBackground first");
    }
    return result;
  }

  bool visited[hd_raw_tof8x8::kGridSize][hd_raw_tof8x8::kGridSize] = {};
  bool foreground[hd_raw_tof8x8::kGridSize][hd_raw_tof8x8::kGridSize] = {};

  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    for (uint8_t column = 0; column < hd_raw_tof8x8::kGridSize; ++column) {
      foreground[row][column] = isForeground(
          row,
          column,
          distances[row * hd_raw_tof8x8::kGridSize + column]);
    }
  }

  uint8_t bestSize = 0;
  uint16_t bestRowSum = 0;
  uint16_t bestColumnSum = 0;
  uint32_t bestDistanceSum = 0;
  uint8_t bestMinimumRow = 0;
  uint8_t bestMaximumRow = 0;
  uint8_t bestMinimumColumn = 0;
  uint8_t bestMaximumColumn = 0;
  uint16_t bestMinimumDistanceMm = 0;
  uint16_t bestMaximumDistanceMm = 0;

  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    for (uint8_t column = 0; column < hd_raw_tof8x8::kGridSize; ++column) {
      if (!foreground[row][column] || visited[row][column]) {
        continue;
      }

      uint8_t rowStack[hd_raw_tof8x8::kPointCount];
      uint8_t columnStack[hd_raw_tof8x8::kPointCount];
      uint8_t stackSize = 0;
      rowStack[stackSize] = row;
      columnStack[stackSize] = column;
      ++stackSize;
      visited[row][column] = true;

      uint8_t size = 0;
      uint16_t rowSum = 0;
      uint16_t columnSum = 0;
      uint32_t distanceSum = 0;
      uint8_t minimumRow = row;
      uint8_t maximumRow = row;
      uint8_t minimumColumn = column;
      uint8_t maximumColumn = column;
      uint16_t minimumDistanceMm = UINT16_MAX;
      uint16_t maximumDistanceMm = 0;

      while (stackSize > 0) {
        --stackSize;
        const uint8_t currentRow = rowStack[stackSize];
        const uint8_t currentColumn = columnStack[stackSize];
        const uint16_t distanceMm =
            distances[currentRow * hd_raw_tof8x8::kGridSize + currentColumn];

        ++size;
        rowSum += currentRow;
        columnSum += currentColumn;
        distanceSum += distanceMm;
        minimumRow = min(minimumRow, currentRow);
        maximumRow = max(maximumRow, currentRow);
        minimumColumn = min(minimumColumn, currentColumn);
        maximumColumn = max(maximumColumn, currentColumn);
        minimumDistanceMm = min(minimumDistanceMm, distanceMm);
        maximumDistanceMm = max(maximumDistanceMm, distanceMm);

        constexpr int8_t rowOffsets[] = {-1, 1, 0, 0};
        constexpr int8_t columnOffsets[] = {0, 0, -1, 1};

        for (uint8_t direction = 0; direction < 4; ++direction) {
          const int8_t neighbourRow = currentRow + rowOffsets[direction];
          const int8_t neighbourColumn =
              currentColumn + columnOffsets[direction];

          if (neighbourRow < 0 ||
              neighbourRow >= hd_raw_tof8x8::kGridSize ||
              neighbourColumn < 0 ||
              neighbourColumn >= hd_raw_tof8x8::kGridSize ||
              !foreground[neighbourRow][neighbourColumn] ||
              visited[neighbourRow][neighbourColumn]) {
            continue;
          }

          const uint16_t neighbourDistanceMm =
              distances[neighbourRow * hd_raw_tof8x8::kGridSize +
                        neighbourColumn];
          const uint16_t newMinimumDistanceMm =
              min(minimumDistanceMm, neighbourDistanceMm);
          const uint16_t newMaximumDistanceMm =
              max(maximumDistanceMm, neighbourDistanceMm);

          if (newMaximumDistanceMm - newMinimumDistanceMm <=
              kFlatnessThresholdMm) {
            visited[neighbourRow][neighbourColumn] = true;
            rowStack[stackSize] = neighbourRow;
            columnStack[stackSize] = neighbourColumn;
            ++stackSize;
          }
        }
      }

      if (size > bestSize) {
        bestSize = size;
        bestRowSum = rowSum;
        bestColumnSum = columnSum;
        bestDistanceSum = distanceSum;
        bestMinimumRow = minimumRow;
        bestMaximumRow = maximumRow;
        bestMinimumColumn = minimumColumn;
        bestMaximumColumn = maximumColumn;
        bestMinimumDistanceMm = minimumDistanceMm;
        bestMaximumDistanceMm = maximumDistanceMm;
      }
    }
  }

  if (bestSize < kMinimumClusterSize) {
    return result;
  }

  const uint16_t averageDistanceMm = bestDistanceSum / bestSize;
  const bool sizeOkay = sizeIsPlausible(bestSize, averageDistanceMm);
  const bool planeOkay = !looksLikeAPlane(
      bestMinimumRow,
      bestMaximumRow,
      bestMinimumColumn,
      bestMaximumColumn);
  const bool flatnessOkay =
      isFlatEnough(bestMinimumDistanceMm, bestMaximumDistanceMm);

  if (verbose) {
    Serial.print("candidate: size=");
    Serial.print(bestSize);
    Serial.print(" dist=");
    Serial.print(averageDistanceMm);
    Serial.print(" | sizeOk=");
    Serial.print(sizeOkay);
    Serial.print(" planeOk=");
    Serial.print(planeOkay);
    Serial.print(" flatOk=");
    Serial.println(flatnessOkay);
  }

  if (!sizeOkay || !planeOkay || !flatnessOkay) {
    return result;
  }

  result.found = true;
  result.clusterSize = bestSize;
  result.averageRow = static_cast<float>(bestRowSum) / bestSize;
  result.averageColumn = static_cast<float>(bestColumnSum) / bestSize;
  result.averageDistanceMm = averageDistanceMm;
  return result;
}

void printWeightResult(const WeightResult& result) {
  if (!result.found) {
    Serial.println("no weight");
    return;
  }

  Serial.print("WEIGHT found | col=");
  Serial.print(result.averageColumn, 1);
  Serial.print(" row=");
  Serial.print(result.averageRow, 1);
  Serial.print(" dist=");
  Serial.print(result.averageDistanceMm);
  Serial.print("mm size=");
  Serial.println(result.clusterSize);
}

namespace {

constexpr uint16_t kRangeObjectMaximumDistanceMm = 2000;
constexpr uint8_t kRangeStableSampleCount = 3;
constexpr uint8_t kPairedDetectionCount = 2;

struct RangeFilterState {
  RangeObjectDetection filtered;
  RangeObjectDetection candidate;
  uint8_t sampleCount;
};

RangeFilterState rangeStates[kRangeDetectionCount] = {};

bool rangeSensorSeesObject(
    const hd_raw_tof::Readings& readings,
    uint8_t sensorIndex) {
  if (sensorIndex >= hd_raw_tof::kSensorCount ||
      !readings[sensorIndex].valid) {
    return false;
  }

  const uint16_t distanceMm = readings[sensorIndex].distanceMm;
  return distanceMm > 0 && distanceMm <= kRangeObjectMaximumDistanceMm;
}

RangeObjectDetection classifyPairedSensors(
    const hd_raw_tof::Readings& readings,
    uint8_t pairIndex) {
  const uint8_t bottomSensor = pairIndex * 2;
  const uint8_t topSensor = bottomSensor + 1;
  const bool bottomSeen = rangeSensorSeesObject(readings, bottomSensor);
  const bool topSeen = rangeSensorSeesObject(readings, topSensor);

  if (!bottomSeen) {
    return {topSeen ? RangeObjectType::unknown : RangeObjectType::none, 0};
  }

  return {
      topSeen ? RangeObjectType::wall : RangeObjectType::weight,
      readings[bottomSensor].distanceMm,
  };
}

RangeObjectDetection classifyWallSensor(
    const hd_raw_tof::Readings& readings,
    uint8_t sensorIndex) {
  if (!rangeSensorSeesObject(readings, sensorIndex)) {
    return {RangeObjectType::none, 0};
  }

  return {RangeObjectType::wall, readings[sensorIndex].distanceMm};
}

void updateRangeFilter(
    RangeFilterState& state,
    const RangeObjectDetection& reading) {
  if (reading.type != state.candidate.type) {
    state.candidate = reading;
    state.sampleCount = 1;
  } else {
    state.candidate.distanceMm = reading.distanceMm;
    if (state.sampleCount < kRangeStableSampleCount) {
      ++state.sampleCount;
    }
  }

  if (state.sampleCount >= kRangeStableSampleCount) {
    state.filtered = state.candidate;
  }
}

}  // namespace

void resetRangeDetections() {
  for (uint8_t detection = 0;
       detection < kRangeDetectionCount;
       ++detection) {
    rangeStates[detection].filtered = {RangeObjectType::none, 0};
    rangeStates[detection].candidate = {RangeObjectType::none, 0};
    rangeStates[detection].sampleCount = 0;
  }
}

void updateRangeDetections(const hd_raw_tof::Readings& readings) {
  for (uint8_t pair = 0; pair < kPairedDetectionCount; ++pair) {
    updateRangeFilter(rangeStates[pair], classifyPairedSensors(readings, pair));
  }

  updateRangeFilter(rangeStates[2], classifyWallSensor(readings, 4));
  updateRangeFilter(rangeStates[3], classifyWallSensor(readings, 5));
}

RangeObjectDetection getRangeDetection(uint8_t detectionIndex) {
  if (detectionIndex >= kRangeDetectionCount) {
    return {RangeObjectType::unknown, 0};
  }

  return rangeStates[detectionIndex].filtered;
}

const char* rangeObjectTypeName(RangeObjectType type) {
  switch (type) {
    case RangeObjectType::none:
      return "NONE";
    case RangeObjectType::weight:
      return "WEIGHT";
    case RangeObjectType::wall:
      return "WALL";
    default:
      return "UNKNOWN";
  }
}

void printRangeDetections() {
  for (uint8_t detectionIndex = 0;
       detectionIndex < kRangeDetectionCount;
       ++detectionIndex) {
    const RangeObjectDetection detection =
        getRangeDetection(detectionIndex);

    if (detectionIndex < kPairedDetectionCount) {
      Serial.print("Pair ");
      Serial.print(detectionIndex);
    } else {
      Serial.print("Wall sensor ");
      Serial.print(detectionIndex + 2);
    }

    Serial.print(": ");
    Serial.print(rangeObjectTypeName(detection.type));

    if (detection.type != RangeObjectType::none &&
        detection.distanceMm > 0) {
      Serial.print(" at ");
      Serial.print(detection.distanceMm);
      Serial.print("mm");
    }

    if (detectionIndex + 1 < kRangeDetectionCount) {
      Serial.print("   ");
    }
  }

  Serial.println();
}

}  // namespace filter_weightDetect
