// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    //TODO: Assign a real CAN ID
    public static final int kIntakeMotorID = 0;
  }

  public static class DriveConstants {
    public static final int kSmartCurrentLimit = 40;
  }

  public static class ShooterConstants {
    //TODO: Assign real CAN ID's for the 2 flywheels
    public static final int kLeftFlywheelID = 0;
    public static final int kRigthFlywheelID = 0;

    public static final double kFlywheelEncoderCPR = 42.0;
    public static final double kAbsoluteEncoderCPR = 4096.0;
    //TODO: Find the flywheel diameter
    public static final double kFlywheelDiameterMeters = 0.09525;
    //TODO: Find gear ratio
    public static final double kFlywheelGearRatio = 6.75;

    public static final double kFlywheelEncoderDistancePerPulse =
        (kFlywheelDiameterMeters * Math.PI) / (kFlywheelEncoderCPR * kFlywheelGearRatio);
  }
}
