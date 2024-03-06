package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.PID;
import frc.robot.TunableDouble;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveModule;

public class JoystickTargetNote extends Command {
  private final DriveSubsystem m_drive;
  private final Limelight m_limelight;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;

  private ProfiledPIDController m_thetaController = new ProfiledPIDController(
    SwerveModule.tunableTeleopTurningPID.get().p(),
    SwerveModule.tunableTeleopTurningPID.get().i(),
    SwerveModule.tunableTeleopTurningPID.get().i(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

  private TunableDouble m_targetVelocityCoefficient =
    new TunableDouble("kTargetCoefficient", VisionConstants.kTargetCoefficient);

  public JoystickTargetNote(DriveSubsystem drive, Limelight limelight,
      Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier) {
    m_drive = drive;
    m_limelight = limelight;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    m_thetaController.reset(0);
    m_thetaController.setTolerance(VisionConstants.kTargetingTolerance);
    m_limelight.lightsOn();
  }

  @Override
  public void execute() {
    updateConstants();
    double x = m_limelight.getX();
    double thetaVelocity =
      m_thetaController.calculate(Units.degreesToRadians(x)) * m_targetVelocityCoefficient.get();
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );
  }

  private void updateConstants() {
    if (SwerveModule.tunableTeleopTurningPID.hasChanged()) {
      PID pid = SwerveModule.tunableTeleopTurningPID.get();
      m_thetaController.setPID(pid.p(), pid.i(), pid.d());
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    m_limelight.lightsOff();
  }
}