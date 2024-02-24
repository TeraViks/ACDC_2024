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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ChamberConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.JoystickTargetNote;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chamber;
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
  private final CameraSubsystem m_cameraSystem = new CameraSubsystem(PhotonVisionConstants.kCameraName1, PhotonVisionConstants.kCameraName2);
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_cameraSystem);
  private final Intake m_intake = new Intake(IntakeConstants.kTopIntakeMotorID, IntakeConstants.kBottomIntakeMotorID, false, IntakeConstants.kPingPort, IntakeConstants.kEchoPort);
  private final Chamber m_chamber = new Chamber(ChamberConstants.kleftChamberMotorID, ChamberConstants.krightChamberMotorID, false, ChamberConstants.kSensorID);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kLeftFlywheelID, ShooterConstants.kRigthFlywheelID, false, true);

  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);

  double reverseFactor = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;

  private static double joystickTransform(double value) {
    double postDeadbandValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    double postDeadbandValueSquared = postDeadbandValue * Math.abs(postDeadbandValue);
    return postDeadbandValueSquared;
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
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))*OIConstants.kMaxMetersPerSec,
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))*OIConstants.kMaxMetersPerSec,
              -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))*OIConstants.kMaxRadPerSec,
              true);
          }, m_robotDrive
        )
      );

      m_chooser.setDefaultOption("Two Note Path", new PathPlannerAuto("two note auto"));
      SmartDashboard.putData(m_chooser);
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kJoystickTargetNoteButton)
      .whileTrue(new JoystickTargetNote(
        m_robotDrive,
        m_limelight,
        () -> reverseFactor * joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis)) * OIConstants.kMaxMetersPerSec,
        () -> reverseFactor * joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis)) * OIConstants.kMaxMetersPerSec
      ));

    // temporary, manual commands for tuning motor speeds
    if (true) { 
      // Intake and Chamber
      new JoystickButton(m_driverController, OIConstants.kB)
        .toggleOnTrue(Commands.startEnd(
          () -> {
            Commands.runOnce(() -> m_intake.startIntake(SmartDashboard.getNumber("Intake Speed (m/s)", 0)))
              .alongWith(Commands.runOnce(() ->m_chamber.moveNote(SmartDashboard.getNumber("Chamber Intake Speed (m/s)", 0))))
              .andThen(Commands.waitUntil(m_chamber::isNoteChambered))
              .andThen(Commands.runOnce(() -> m_chamber.moveNote(0)));
          },
          () -> {
            m_intake.stopIntake();
            m_chamber.moveNote(0);
          }
        ));

        // Chamber and Shooter
        new JoystickButton(m_driverController, OIConstants.kA)
          .toggleOnTrue(Commands.startEnd(
            () -> {
              Commands.runOnce(() -> m_shooter.shoot(SmartDashboard.getNumber("Shooter Speed Right (m/s)", 0), SmartDashboard.getNumber("Shooter Speed Left (m/s)", 0)))
                .andThen(Commands.waitUntil(m_shooter::isReadyToShoot))
                .andThen(Commands.runOnce(() -> m_chamber.moveNote(SmartDashboard.getNumber("Chamber Shoot Speed (m/s)", 0))));
            },
            () -> {
              m_shooter.stopShooter();
              m_chamber.moveNote(0);
            }
          ));
    }
  }

  public Command getAutonomousCommand() {
    Command selectedCommand = m_chooser.getSelected();
    
    if (selectedCommand instanceof PathPlannerAuto) {
      PathPlannerAuto selectedAuto = (PathPlannerAuto)selectedCommand;
      Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(selectedAuto.getName());
      Pose2d transformedPose = mirrorPose2d(startingPose);
      m_robotDrive.initOdometry(transformedPose);
    }

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