#include "weight_detect.h"

#include <math.h>

namespace
{
uint16_t backgroundMap[GRID_SIZE][GRID_SIZE];
bool backgroundCalibrated = false;

bool isForeground(int row, int column, uint16_t measured)
{
  if (measured == 0 || measured == 4000)
  {
    return false;
  }

  const uint16_t background = backgroundMap[row][column];
  if (background == 0 || background == 4000)
  {
    return true;
  }

  return static_cast<int16_t>(background) - static_cast<int16_t>(measured) >
         NEAR_DELTA_MM;
}

float expectedWidthPixels(float distanceMillimeters)
{
  const float halfPixelAngleRadians =
      (HFOV_DEG / (2.0f * GRID_SIZE)) * DEG_TO_RAD;
  float millimetersPerPixel =
      2.0f * distanceMillimeters * tanf(halfPixelAngleRadians);

  if (millimetersPerPixel < 1.0f)
  {
    millimetersPerPixel = 1.0f;
  }

  return WEIGHT_DIAMETER_MM / millimetersPerPixel;
}

bool sizeIsPlausible(int clusterSize, float distanceMillimeters)
{
  const float expectedWidth = expectedWidthPixels(distanceMillimeters);
  const float minimumSize = max(expectedWidth * 0.5f,
                                static_cast<float>(MIN_CLUSTER_SIZE));
  const float maximumSize = max(expectedWidth * expectedWidth * 4.0f, 6.0f);
  return clusterSize >= minimumSize && clusterSize <= maximumSize;
}

bool looksLikePlane(int minRow, int maxRow, int minColumn, int maxColumn)
{
  const bool touchesEdge = minRow == 0 || maxRow == GRID_SIZE - 1 ||
                           minColumn == 0 || maxColumn == GRID_SIZE - 1;
  const int rowSpan = maxRow - minRow + 1;
  const int columnSpan = maxColumn - minColumn + 1;
  return touchesEdge && (rowSpan > 4 || columnSpan > 4);
}
} // namespace

void calibrateBackground(uint16_t distances[GRID_SIZE * GRID_SIZE])
{
  for (int row = 0; row < GRID_SIZE; row++)
  {
    for (int column = 0; column < GRID_SIZE; column++)
    {
      backgroundMap[row][column] = distances[row * GRID_SIZE + column];
    }
  }

  backgroundCalibrated = true;
}

void calibrateBackgroundAveraged(
    uint16_t frames[][GRID_SIZE * GRID_SIZE], int numFrames)
{
  uint32_t sums[GRID_SIZE][GRID_SIZE] = {};

  for (int frame = 0; frame < numFrames; frame++)
  {
    for (int row = 0; row < GRID_SIZE; row++)
    {
      for (int column = 0; column < GRID_SIZE; column++)
      {
        sums[row][column] += frames[frame][row * GRID_SIZE + column];
      }
    }
  }

  for (int row = 0; row < GRID_SIZE; row++)
  {
    for (int column = 0; column < GRID_SIZE; column++)
    {
      backgroundMap[row][column] = sums[row][column] / numFrames;
    }
  }

  backgroundCalibrated = true;
}

bool isBackgroundCalibrated()
{
  return backgroundCalibrated;
}

WeightResult detectWeight(
    uint16_t distances[GRID_SIZE * GRID_SIZE], bool verbose, Print &output)
{
  WeightResult result = {false, 0, 0, 0, 0};
  if (!backgroundCalibrated)
  {
    return result;
  }

  bool visited[GRID_SIZE][GRID_SIZE] = {};
  bool isNear[GRID_SIZE][GRID_SIZE];

  for (int row = 0; row < GRID_SIZE; row++)
  {
    for (int column = 0; column < GRID_SIZE; column++)
    {
      isNear[row][column] =
          isForeground(row, column, distances[row * GRID_SIZE + column]);
    }
  }

  int bestSize = 0;
  float bestRowSum = 0;
  float bestColumnSum = 0;
  int bestDistanceSum = 0;
  int bestMinRow = 0;
  int bestMaxRow = 0;
  int bestMinColumn = 0;
  int bestMaxColumn = 0;
  uint16_t bestMinDistance = 0;
  uint16_t bestMaxDistance = 0;

  for (int row = 0; row < GRID_SIZE; row++)
  {
    for (int column = 0; column < GRID_SIZE; column++)
    {
      if (!isNear[row][column] || visited[row][column])
      {
        continue;
      }

      int rowStack[GRID_SIZE * GRID_SIZE];
      int columnStack[GRID_SIZE * GRID_SIZE];
      int stackPointer = 0;
      rowStack[stackPointer] = row;
      columnStack[stackPointer++] = column;
      visited[row][column] = true;

      int size = 0;
      float rowSum = 0;
      float columnSum = 0;
      int distanceSum = 0;
      int minRow = row;
      int maxRow = row;
      int minColumn = column;
      int maxColumn = column;
      uint16_t minDistance = UINT16_MAX;
      uint16_t maxDistance = 0;

      while (stackPointer > 0)
      {
        stackPointer--;
        const int currentRow = rowStack[stackPointer];
        const int currentColumn = columnStack[stackPointer];
        const uint16_t distance =
            distances[currentRow * GRID_SIZE + currentColumn];

        size++;
        rowSum += currentRow;
        columnSum += currentColumn;
        distanceSum += distance;
        minRow = min(minRow, currentRow);
        maxRow = max(maxRow, currentRow);
        minColumn = min(minColumn, currentColumn);
        maxColumn = max(maxColumn, currentColumn);
        minDistance = min(minDistance, distance);
        maxDistance = max(maxDistance, distance);

        const int rowOffsets[] = {-1, 1, 0, 0};
        const int columnOffsets[] = {0, 0, -1, 1};

        for (int direction = 0; direction < 4; direction++)
        {
          const int nextRow = currentRow + rowOffsets[direction];
          const int nextColumn = currentColumn + columnOffsets[direction];

          if (nextRow < 0 || nextRow >= GRID_SIZE || nextColumn < 0 ||
              nextColumn >= GRID_SIZE || !isNear[nextRow][nextColumn] ||
              visited[nextRow][nextColumn])
          {
            continue;
          }

          const uint16_t neighborDistance =
              distances[nextRow * GRID_SIZE + nextColumn];
          const uint16_t newMinimum = min(minDistance, neighborDistance);
          const uint16_t newMaximum = max(maxDistance, neighborDistance);

          if (newMaximum - newMinimum <= FLATNESS_THRESHOLD_MM)
          {
            visited[nextRow][nextColumn] = true;
            rowStack[stackPointer] = nextRow;
            columnStack[stackPointer++] = nextColumn;
          }
        }
      }

      if (size > bestSize)
      {
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

  if (bestSize < MIN_CLUSTER_SIZE)
  {
    return result;
  }

  const int averageDistance = bestDistanceSum / bestSize;
  const bool sizeOkay = sizeIsPlausible(bestSize, averageDistance);
  const bool planeOkay =
      !looksLikePlane(bestMinRow, bestMaxRow, bestMinColumn, bestMaxColumn);
  const bool flatOkay =
      bestMaxDistance - bestMinDistance <= FLATNESS_THRESHOLD_MM;

  if (verbose)
  {
    output.print("candidate: size=");
    output.print(bestSize);
    output.print(" dist=");
    output.print(averageDistance);
    output.print(" | sizeOk=");
    output.print(sizeOkay);
    output.print(" planeOk=");
    output.print(planeOkay);
    output.print(" flatOk=");
    output.println(flatOkay);
  }

  if (!sizeOkay || !planeOkay || !flatOkay)
  {
    return result;
  }

  result.found = true;
  result.clusterSize = bestSize;
  result.avgRow = bestRowSum / bestSize;
  result.avgCol = bestColumnSum / bestSize;
  result.avgDistMM = averageDistance;
  return result;
}

void printResult(const WeightResult &result, Print &output)
{
  if (!result.found)
  {
    output.println("no weight");
    return;
  }

  output.print("WEIGHT found | col=");
  output.print(result.avgCol, 1);
  output.print(" row=");
  output.print(result.avgRow, 1);
  output.print(" dist=");
  output.print(result.avgDistMM);
  output.print("mm size=");
  output.println(result.clusterSize);
}
