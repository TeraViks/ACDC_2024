package frc.robot.utilities;

import java.util.Arrays;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public final class ShooterInterp {
  static class PosEntry{double distance; double speed; public PosEntry(double dist, double pow) {distance = dist; speed = pow;}}
  static final PosEntry[] posTable = {
    new PosEntry(2.5, 22.0),
    new PosEntry(2.7, 22.0),
    new PosEntry(2.9, 23.5),
    new PosEntry(3.05, 22.7),
    new PosEntry(3.12, 21.2),
    new PosEntry(3.22, 20.8)
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
    if (ShooterConstants.kEnableManualSpeed) {
      return Shooter.manualSpeed.get();
    }
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