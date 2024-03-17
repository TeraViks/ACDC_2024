package frc.robot.utilities;

import java.util.Arrays;

public final class ShooterInterp {
  static class PosEntry{double distance; double speed; public PosEntry(double dist, double pow) {distance = dist; speed = pow;}}
  static final PosEntry[] posTable = {
    new PosEntry(2.9, 20.7),
    new PosEntry(3.15, 21),
    new PosEntry(3.45, 20.5),
    new PosEntry(3.52, 21.3)
  };

  public static double getMaxDistance() {
    return posTable[posTable.length-1].distance;
  }

  public static int distanceIndex(double distance) {
    assert(distance >= 0);
    int index = Arrays.binarySearch(posTable, new PosEntry(distance, 0), (a,b) -> Double.compare(a.distance, b.distance));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public static double distanceToSpeed(double distance) {
    int index = distanceIndex(distance);
    if (index == 0) {
      return posTable[index].speed;
    } else if (index == posTable.length) {
      return posTable[posTable.length-1].speed;
    } else if (posTable[index].distance == distance) {
      return posTable[index].speed;
    }
    // Linear Interpolation
    else {
      double d0 = posTable[index-1].distance;
      double d1 = posTable[index].distance;
      double s0 = posTable[index-1].speed;
      double s1 = posTable[index].speed;
      double scaler = (distance-d0) / (d1 - d0);
      double speedRange = s1 - s0;
      return s0 + scaler*speedRange;
    }
  }
}