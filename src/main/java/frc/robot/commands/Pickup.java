// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.NoteConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class Pickup extends SequentialCommandGroup {
  public Pickup(DriveSubsystem drive, Limelight limelight, Chamber chamber) {
    JoystickTargetNote joystickTargetNote = new JoystickTargetNote(drive, limelight, () -> 0.0, () -> 0.0);
    addCommands(
      Commands.runOnce(() -> chamber.chamberNote())
      .andThen(
        joystickTargetNote.until(joystickTargetNote::isFacingNote)
        .andThen(
          Commands.run(() -> drive.drive(NoteConstants.kPickupSpeed, 0.0, 0.0, false))
          .until(chamber::isNoteChambered)
          .withTimeout(NoteConstants.kTimeout)
        )
      ).finallyDo(() -> { if (!chamber.isNoteChambered()) chamber.cancelChambering(); })
    );
  }
}
