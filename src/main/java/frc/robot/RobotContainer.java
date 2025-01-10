// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.commands.JoystickTargetNote;
import frc.robot.commands.JoystickTargetSpeaker;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.TargetSpeaker;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Limelight m_limelight = LimelightConstants.kEnable ? new Limelight() : null;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final CameraSubsystem m_cameraSystem = PhotonVisionConstants.kEnable ? new CameraSubsystem() : null;
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_cameraSystem);
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  double m_reverseFactor = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;

  private static double joystickTransform(double value) {
    double transformedValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    if (DriveConstants.kSquareInputs) {
      transformedValue = transformedValue * Math.abs(transformedValue);
    }
    return transformedValue;
  }

  private double getXSpeedInput() {
    // The Y axis on the controller is inverted! Pushing forward (up) generates a negative value.
    // Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))
      * OIConstants.kMaxMetersPerSec;
  }

  private double getYSpeedInput() {
    // The X axis on the controller behaves as expected (right is positive), but we're using it for
    // Y axis control, where left is positive. Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))
      * OIConstants.kMaxMetersPerSec;
  }

  private double getRotationSpeedInput() {
    // Moving the joystick to the right causes positive input, which we negate in order to rotate
    // clockwise.
    return -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))
      * OIConstants.kMaxRadPerSec;
  }

  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> {
            m_robotDrive.drive(
              getXSpeedInput(),
              getYSpeedInput(),
              getRotationSpeedInput(),
              true);
          }, m_robotDrive
        )
      );

      m_chooser.setDefaultOption("Empty Auto", new PathPlannerAuto("Empty Auto"));
      m_chooser.addOption("Zero Note A", new PathPlannerAuto("zero note A"));
      m_chooser.addOption("Zero Note B", new PathPlannerAuto("Zero note B"));
      m_chooser.addOption("Zero Note C", new PathPlannerAuto("zero note C"));
      m_chooser.addOption("One Note A", new PathPlannerAuto("one note A"));
      m_chooser.addOption("One Note B", new PathPlannerAuto("one note B"));
      m_chooser.addOption("One Note C", new PathPlannerAuto("one note C"));
      m_chooser.addOption("A Forward and Back", new PathPlannerAuto("A zero"));
      m_chooser.addOption("B Forward and Back", new PathPlannerAuto("B zero"));
      m_chooser.addOption("C Forward and Back", new PathPlannerAuto("C zero"));
      m_chooser.addOption("Delayed One Note C", new PathPlannerAuto("delayed one note C"));
      m_chooser.addOption("Under Stage C", new PathPlannerAuto("under stage C"));
      SmartDashboard.putData(m_chooser);
  }

  public void setPIDSlotID(ClosedLoopSlot slotID) {
    m_robotDrive.setPIDSlotID(slotID);
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kZeroGyro)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> {
        m_robotDrive.zeroGyro();
      }, m_robotDrive
    ));

    if (LimelightConstants.kEnable) {
      new JoystickButton(m_driverController, OIConstants.kJoystickTargetNoteButton)
        .debounce(OIConstants.kDebounceSeconds)
        .whileTrue(new JoystickTargetNote(
          m_robotDrive,
          m_limelight,
          () -> getXSpeedInput(),
          () -> getYSpeedInput()
        ));
    }

    new JoystickButton(m_driverController, OIConstants.kJoystickTargetSpeakerButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new JoystickTargetSpeaker(
        m_robotDrive,
        m_driverController,
        () -> getXSpeedInput(),
        () -> getYSpeedInput(),
        new TargetSpeaker()
      ));
  }

  public Command getAutonomousCommand() {
    Command selectedCommand = m_chooser.getSelected();
    return selectedCommand;
  }
}
