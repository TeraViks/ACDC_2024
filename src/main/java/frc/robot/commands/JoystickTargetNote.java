package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.TunableConstant;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class JoystickTargetNote extends Command {
  private final DriveSubsystem m_drive;
  private final Limelight m_limelight;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;

  private ProfiledPIDController thetaController = new ProfiledPIDController(
      SwerveModuleConstants.kTurningPID.p(),
      SwerveModuleConstants.kTurningPID.i(),
      SwerveModuleConstants.kTurningPID.d(),
      new TrapezoidProfile.Constraints(
        DriveConstants.kMaxAngularSpeedRadiansPerSecond,
        DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

  private TunableConstant targetVelocityCoefficient = new TunableConstant("kTargetCoefficient", VisionConstants.kTargetCoefficient);
  private TunableConstant maxAngularSpeed = new TunableConstant("Max Angular Speed", DriveConstants.kMaxAngularSpeedRadiansPerSecond);
  private TunableConstant maxAngularAcceleration = new TunableConstant("Max Angular Acceleration", DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
  private TunableConstant p = new TunableConstant("kP", SwerveModuleConstants.kTurningPID.p());
  private TunableConstant i = new TunableConstant("kI", SwerveModuleConstants.kTurningPID.i());
  private TunableConstant d = new TunableConstant("kD", SwerveModuleConstants.kTurningPID.d());

  public JoystickTargetNote(DriveSubsystem drive, Limelight limelight, Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier) {
    m_drive = drive;
    m_limelight = limelight;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    thetaController.reset(0);
    thetaController.setTolerance(VisionConstants.kTargetingTolerance);
    m_limelight.lightsOn();
  }

  @Override
  public void execute() {
    updateConstants();
    double x = m_limelight.getX();
    double thetaVelocity = thetaController.calculate(Units.degreesToRadians(x)) * targetVelocityCoefficient.get();
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );
  }

  private void updateConstants() {
    if (maxAngularSpeed.hasChanged() || maxAngularAcceleration.hasChanged()) {
      thetaController.setConstraints(new TrapezoidProfile.Constraints(maxAngularSpeed.get(), maxAngularAcceleration.get()));
    }

    if (p.hasChanged() || i.hasChanged() || d.hasChanged()) {
      thetaController.setPID(p.get(), i.get(), d.get());
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    m_limelight.lightsOff();
  }
}
