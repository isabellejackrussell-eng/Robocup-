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
constexpr uint16_t kCloseRangeDistanceMm = 80;
constexpr uint16_t kMaximumWeightDistanceMm = 300;

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
  // The recorded sensor data shows that a 50 mm weight closer than 80 mm is
  // represented by a stable two-pixel vertical cluster, rather than the much
  // larger footprint predicted by the ideal field-of-view geometry.
  const float minimumSize = distanceMm < kCloseRangeDistanceMm
      ? static_cast<float>(kMinimumClusterSize)
      : max(expectedWidth * 0.5f, static_cast<float>(kMinimumClusterSize));
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

  uint8_t largestCandidateSize = 0;
  uint16_t largestCandidateDistanceMm = 0;
  bool largestCandidateDistanceOkay = false;
  bool largestCandidateSizeOkay = false;
  bool largestCandidatePlaneOkay = false;
  bool largestCandidateFlatnessOkay = false;

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

      if (size < kMinimumClusterSize) {
        continue;
      }

      const uint16_t averageDistanceMm = distanceSum / size;
      const bool distanceOkay =
          averageDistanceMm <= kMaximumWeightDistanceMm;
      const bool sizeOkay = sizeIsPlausible(size, averageDistanceMm);
      const bool planeOkay = !looksLikeAPlane(
          minimumRow,
          maximumRow,
          minimumColumn,
          maximumColumn);
      const bool flatnessOkay =
          isFlatEnough(minimumDistanceMm, maximumDistanceMm);

      if (size > largestCandidateSize) {
        largestCandidateSize = size;
        largestCandidateDistanceMm = averageDistanceMm;
        largestCandidateDistanceOkay = distanceOkay;
        largestCandidateSizeOkay = sizeOkay;
        largestCandidatePlaneOkay = planeOkay;
        largestCandidateFlatnessOkay = flatnessOkay;
      }

      // Select the largest valid weight candidate. Previously the largest raw
      // cluster was selected first and only validated afterwards, allowing a
      // distant background artifact to hide an equally sized nearby weight.
      if (distanceOkay && sizeOkay && planeOkay && flatnessOkay &&
          size > bestSize) {
        bestSize = size;
        bestRowSum = rowSum;
        bestColumnSum = columnSum;
        bestDistanceSum = distanceSum;
      }
    }
  }

  if (bestSize < kMinimumClusterSize) {
    if (verbose && largestCandidateSize >= kMinimumClusterSize) {
      Serial.print("rejected candidate: size=");
      Serial.print(largestCandidateSize);
      Serial.print(" dist=");
      Serial.print(largestCandidateDistanceMm);
      Serial.print(" | distanceOk=");
      Serial.print(largestCandidateDistanceOkay);
      Serial.print(" sizeOk=");
      Serial.print(largestCandidateSizeOkay);
      Serial.print(" planeOk=");
      Serial.print(largestCandidatePlaneOkay);
      Serial.print(" flatOk=");
      Serial.println(largestCandidateFlatnessOkay);
    }
    return result;
  }

  const uint16_t averageDistanceMm = bestDistanceSum / bestSize;
  if (verbose) {
    Serial.print("accepted candidate: size=");
    Serial.print(bestSize);
    Serial.print(" dist=");
    Serial.print(averageDistanceMm);
    Serial.println(" | distanceOk=1 sizeOk=1 planeOk=1 flatOk=1");
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

bool distancesMatch(uint16_t firstMm, uint16_t secondMm) {
  const uint16_t differenceMm = firstMm > secondMm
      ? firstMm - secondMm
      : secondMm - firstMm;
  return differenceMm <= kWallDistanceToleranceMm;
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

  const bool sameObjectAtBothHeights =
      topSeen && distancesMatch(
          readings[bottomSensor].distanceMm,
          readings[topSensor].distanceMm);

  return {
      sameObjectAtBothHeights ? RangeObjectType::wall : RangeObjectType::weight,
      readings[bottomSensor].distanceMm,
  };
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

    Serial.print("ToF set ");
    Serial.print(detectionIndex);

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

WeightIdentification identifyWeight(
    const hd_raw_tof::Readings& readings,
    const WeightResult& matrixWeight) {
  constexpr uint8_t kRightPairIndex = 0;
  constexpr uint8_t kLeftPairIndex = 1;

  // The top sensors from the right and left pairs can confirm that an 8x8
  // candidate is a tall wall.
  constexpr uint8_t kTopOrWallSensorIndices[] = {1, 3};

  // updateRangeDetections() supplies the debounced three-sample result used
  // here. Keep the raw readings only for the top-sensor wall comparison.
  const RangeObjectDetection rightDetection =
      getRangeDetection(kRightPairIndex);
  const RangeObjectDetection leftDetection =
      getRangeDetection(kLeftPairIndex);

  const bool rightWeight = rightDetection.type == RangeObjectType::weight;
  const bool leftWeight = leftDetection.type == RangeObjectType::weight;

  bool matrixMatchesWall = false;
  if (matrixWeight.found) {
    for (uint8_t sensorIndex : kTopOrWallSensorIndices) {
      if (rangeSensorSeesObject(readings, sensorIndex) &&
          distancesMatch(
              matrixWeight.averageDistanceMm,
              readings[sensorIndex].distanceMm)) {
        matrixMatchesWall = true;
        break;
      }
    }
  }

  const bool centreWeight = matrixWeight.found && !matrixMatchesWall;

  uint32_t distanceSumMm = 0;
  uint8_t distanceCount = 0;

  if (rightWeight) {
    distanceSumMm += rightDetection.distanceMm;
    ++distanceCount;
  }
  if (centreWeight) {
    distanceSumMm += matrixWeight.averageDistanceMm;
    ++distanceCount;
  }
  if (leftWeight) {
    distanceSumMm += leftDetection.distanceMm;
    ++distanceCount;
  }

  const uint16_t combinedDistanceMm = distanceCount > 0
      ? static_cast<uint16_t>(distanceSumMm / distanceCount)
      : static_cast<uint16_t>(0);

  WeightIdentification identification = {
      distanceCount > 0 ? WeightState::found : WeightState::notFound,
      combinedDistanceMm,
      rightWeight
          ? rightDetection.distanceMm
          : static_cast<uint16_t>(0),
      centreWeight
          ? matrixWeight.averageDistanceMm
          : static_cast<uint16_t>(0),
      leftWeight
          ? leftDetection.distanceMm
          : static_cast<uint16_t>(0),
      centreWeight ? matrixWeight.averageColumn : -1.0f,
      rightWeight,
      centreWeight,
      leftWeight,
  };

  return identification;
}

bool weightFound(const WeightIdentification& identification) {
  return identification.state == WeightState::found;
}

void printWeightIdentification(const WeightIdentification& identification) {
  if (!weightFound(identification)) {
    Serial.println("Weight identification: NOT FOUND");
    return;
  }

  Serial.print("Weight identification: FOUND at ");
  Serial.print(identification.distanceMm);
  Serial.print("mm | seen by:");

  if (identification.seenByRightTof) {
    Serial.print(" RIGHT(");
    Serial.print(identification.rightDistanceMm);
    Serial.print("mm)");
  }
  if (identification.seenBy8x8) {
    Serial.print(" CENTRE(column=");
    Serial.print(identification.column, 1);
    Serial.print(", distance=");
    Serial.print(identification.centreDistanceMm);
    Serial.print("mm)");
  }
  if (identification.seenByLeftTof) {
    Serial.print(" LEFT(");
    Serial.print(identification.leftDistanceMm);
    Serial.print("mm)");
  }

  Serial.println();
}

WeightIdentification runWeightDetectionTest() {
  Serial.println();
  Serial.println("========== WEIGHT FILTER TEST ==========");

  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  updateRangeDetections(readings);

  for (uint8_t set = 0; set < kPairedDetectionCount; ++set) {
    Serial.print("[RAW ToF set ");
    Serial.print(set);
    Serial.print(set == 0 ? " / RIGHT] " : " / LEFT] ");

    for (uint8_t sensorInSet = 0; sensorInSet < 2; ++sensorInSet) {
      const uint8_t sensor = set * 2 + sensorInSet;
      Serial.print(sensorInSet == 0 ? "bottom=" : " top=");

      if (!hd_raw_tof::isSensorInitialised(sensor)) {
        Serial.print("NOT_INITIALISED");
      } else if (readings[sensor].timedOut) {
        Serial.print("TIMEOUT");
      } else if (!readings[sensor].valid) {
        Serial.print("INVALID");
      } else {
        Serial.print(readings[sensor].distanceMm);
        Serial.print("mm");
      }
    }

    Serial.println();
  }

  Serial.print("[FILTERED PAIRS] ");
  printRangeDetections();

  WeightIdentification noWeight = {
      WeightState::notFound,
      0,
      0,
      0,
      0,
      -1.0f,
      false,
      false,
      false,
  };

  hd_raw_tof8x8::Frame frame;
  if (!hd_raw_tof8x8::readFrame(frame)) {
    Serial.println("[RAW 8x8] READ FAILED");
    printWeightIdentification(noWeight);
    Serial.println("========================================");
    return noWeight;
  }

  Serial.println("[RAW 8x8 / mm]");
  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    Serial.print("row ");
    Serial.print(row);
    Serial.print(": ");
    for (uint8_t column = 0;
         column < hd_raw_tof8x8::kGridSize;
         ++column) {
      Serial.print(frame[row * hd_raw_tof8x8::kGridSize + column]);
      if (column + 1 < hd_raw_tof8x8::kGridSize) {
        Serial.print('\t');
      }
    }
    Serial.println();
  }

  Serial.print("[8x8 FILTER] ");
  const WeightResult matrixWeight = detectWeight(frame, true);
  printWeightResult(matrixWeight);

  Serial.print("[COMBINED FILTER] ");
  const WeightIdentification identification =
      identifyWeight(readings, matrixWeight);
  printWeightIdentification(identification);
  Serial.println("========================================");
  return identification;
}

}  // namespace filter_weightDetect
