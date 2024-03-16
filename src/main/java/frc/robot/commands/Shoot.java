// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
  public Shoot(DriveSubsystem drive, Chamber chamber, Shooter shooter, GenericHID driverController) {
    JoystickTargetSpeaker joytickTargetSpeaker = new JoystickTargetSpeaker(drive, chamber, shooter, driverController, () -> 0.0, () -> 0.0);
    addCommands(
      joytickTargetSpeaker.until(joytickTargetSpeaker::isFacingSpeaker),
      Commands.runOnce(
        () -> {
          double speed = ShooterConstants.kShootingSpeed;
          chamber.shootNote(speed);
        },
        chamber, shooter
      )
    );
  }
}
