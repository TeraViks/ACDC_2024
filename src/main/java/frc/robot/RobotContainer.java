// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.JoystickTargetNote;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Limelight m_limelight = new Limelight();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  // private final CameraSubsystem m_cameraSystem = new CameraSubsystem();
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Chamber m_chamber = new Chamber(m_intake, m_shooter);
  private final Climber m_climber = new Climber();
  private final TunableConstant m_shootingSpeed =
    new TunableConstant("ShootingSpeed", ShooterConstants.kShootingSpeed);
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  double m_reverseFactor = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;

  private static double joystickTransform(double value) {
    double postDeadbandValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    double postDeadbandValueSquared = postDeadbandValue * Math.abs(postDeadbandValue);
    return postDeadbandValueSquared;
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
    return joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))
      * OIConstants.kMaxRadPerSec;
  }

  private double getLeftClimbInput() {
    return joystickTransform(-m_operatorController.getRawAxis(OIConstants.kLeftJoyYAxis));
  }

  private double getRightClimbInput() {
    return joystickTransform(-m_operatorController.getRawAxis(OIConstants.kRightJoyYAxis));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("ShooterCommand", new ShooterCommand());
    NamedCommands.registerCommand("PickupCommand", new PickupCommand());
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

      m_climber.setDefaultCommand(
        new RunCommand(
          () -> {
            m_climber.startClimber(
              getLeftClimbInput(),
              getRightClimbInput()
            );
          }, m_climber
        )
      );

      m_chooser.setDefaultOption("Empty Auto", new PathPlannerAuto("Empty Auto"));
      m_chooser.addOption("Zero Note A", new PathPlannerAuto("zero note A"));
      m_chooser.addOption("Zero Note B", new PathPlannerAuto("Zero note B"));
      m_chooser.addOption("Zero Note C", new PathPlannerAuto("zero note C"));
      SmartDashboard.putData(m_chooser);
  }

  public void setPIDSlotID(int slotID) {
    m_robotDrive.setPIDSlotID(slotID);
  }

  public void initializePreloaded() {
    m_chamber.initializePreloaded();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kJoystickTargetNoteButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new JoystickTargetNote(
        m_robotDrive,
        m_limelight,
        () -> getXSpeedInput(),
        () -> getYSpeedInput()
      ));

    new JoystickButton(m_operatorController, OIConstants.kIntakeOn)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(
        () -> m_chamber.chamberNote()
      ));

    new JoystickButton(m_operatorController, OIConstants.kIntakeOff)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(
        () -> {
          m_chamber.cancelChambering();
          m_intake.stopIntake();
        }
      ));

    new JoystickButton(m_operatorController, OIConstants.kIntakeReverse)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(
        () -> m_intake.reverseIntake()
      ));

    new JoystickButton(m_operatorController, OIConstants.kEjectButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(
        () -> m_chamber.ejectNote(),
        m_chamber, m_shooter));

    new JoystickButton(m_driverController, OIConstants.kShootButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(
        () -> {
          double speed = m_shootingSpeed.get();
          m_chamber.shootNote(speed, speed);
        },
        m_chamber, m_shooter
      ));

    // temporary, manual commands for tuning motor speeds
    if (false) {
      // Intake and Chamber
      new JoystickButton(m_driverController, OIConstants.kB)
        .debounce(OIConstants.kDebounceSeconds)
        .toggleOnTrue(Commands.startEnd(
          () -> {
            m_chamber.chamberNote();
          },
          () -> {
            m_chamber.cancelChambering();
          },
          m_intake, m_chamber
        )
      );

      // Chamber and Shooter
      new JoystickButton(m_driverController, OIConstants.kA)
        .debounce(OIConstants.kDebounceSeconds)
        .toggleOnTrue(Commands.startEnd(
          () -> {
            m_chamber.chamberNote();
            m_shooter.idleShooter();
          },
          () -> {
            m_chamber.cancelChambering();
            m_shooter.stopShooter();
          },
          m_chamber, m_shooter
        ));
    }
  }

  public Command getAutonomousCommand() {
    Command selectedCommand = m_chooser.getSelected();
    Pose2d initialPose;
    
    if (selectedCommand instanceof PathPlannerAuto) {
      PathPlannerAuto selectedAuto = (PathPlannerAuto)selectedCommand;
      Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(selectedAuto.getName());
      initialPose = startingPose;
    } else {
      initialPose = new Pose2d();
    }
    Pose2d transformedPose = mirrorPose2d(initialPose);
    m_robotDrive.initOdometry(transformedPose);
    return selectedCommand;
  }

  private Pose2d mirrorPose2d(Pose2d pose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return pose;
    }
    return new Pose2d(
      FieldConstants.kMaxX - pose.getX(),
      pose.getY(),
      new Rotation2d(Math.PI - pose.getRotation().getRadians())
    );
  }
}
