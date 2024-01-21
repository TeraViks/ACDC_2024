// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

public class ShooterInterp extends SubsystemBase {
  static class PosEntry{double distance; double speed; public PosEntry(double dist, double pow) {distance = dist; speed = pow;}}
  static final PosEntry[] posTable = {};
  /** Creates a new ShooterInterp. */
  public ShooterInterp() {}

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
