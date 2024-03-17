package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.NoteConstants;
import frc.robot.PIDF;
import frc.robot.TunablePIDF;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class JoystickTargetNote extends Command {
  private static final TunablePIDF targetTurningPIDF =
    new TunablePIDF("Note.turningPIDF", NoteConstants.kTurningPIDF);

  private final DriveSubsystem m_drive;
  private final Limelight m_limelight;
  private final Supplier<Double> m_xVelocitySupplier;
  private final Supplier<Double> m_yVelocitySupplier;

  private ProfiledPIDController m_thetaController = new ProfiledPIDController(
    targetTurningPIDF.get().p(),
    targetTurningPIDF.get().i(),
    targetTurningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared),
    Constants.kDt);

  public JoystickTargetNote(DriveSubsystem drive, Limelight limelight,
      Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier) {
    m_drive = drive;
    m_limelight = limelight;
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelocitySupplier = yVelocitySupplier;

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.setTolerance(NoteConstants.kAngularTolerance);

    addRequirements(drive, limelight);
  }

  private double getX() {
    return Units.degreesToRadians(m_limelight.getX());
  }

  @Override
  public void initialize() {
    m_thetaController.reset(getX(), m_drive.getAngularVelocity());
    m_limelight.lightsOn();
  }

  @Override
  public void execute() {
    updateConstants();
    double thetaVelocity = m_thetaController.calculate(getX());
    m_drive.drive(
      m_xVelocitySupplier.get(),
      m_yVelocitySupplier.get(),
      thetaVelocity,
      true
    );
  }

  private void updateConstants() {
    if (targetTurningPIDF.hasChanged()) {
      PIDF pidf = targetTurningPIDF.get();
      m_thetaController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    m_limelight.lightsOff();
  }
}
