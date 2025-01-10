package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController; 

import java.io.IOException;
import java.io.UncheckedIOException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ChamberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.TunableDouble;

public class Chamber extends SubsystemBase {
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;
  private final SparkClosedLoopController m_leftPIDController;
  private final SparkClosedLoopController m_rightPIDController;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final DigitalInput m_noteDetectedSensor;
  private final SendableChooser<State> m_chooser = new SendableChooser<>();

  private final TunableDouble m_intakingSpeed =
    new TunableDouble("Chamber.intakingSpeed", ChamberConstants.kIntakingSpeed);

  private enum State {
    // The chamber controls the intake/chamber/shooter such that they progress in lockstep through
    // the following states.
    //             Intake   Chamber  Shooter
    EMPTY,      // stopped  stopped  stopped
    CHAMBERING, // intaking intaking stopped
    ALIGNING,   // stopped  aligning stopped
    CHAMBERED,  // stopped  stopped  idling
    SHOOTING,   // stopped  stopped  revving
    EJECTING,   // stopped  stopped  revving
    CLEARING    // stopped  shooting revving
  }
  private State m_state;
  private double m_shootTimeSeconds = 0.0;

  public Chamber(Intake intake, Shooter shooter) {
    m_chooser.setDefaultOption("Empty", State.EMPTY);
    m_chooser.addOption("Preloaded", State.CHAMBERED);
    SmartDashboard.putData(m_chooser);
    m_state = State.EMPTY;
    
    m_leftMotor = new SparkMax(ChamberConstants.kLeftMotorID, MotorType.kBrushless);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(ChamberConstants.kLeftMotorReversed)
      .smartCurrentLimit(ChamberConstants.kCurrentLimit)
      .idleMode(IdleMode.kBrake);
    leftMotorConfig.encoder
      .velocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);
    leftMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    ChamberConstants.kLeftPIDF.controllerSet(leftMotorConfig.closedLoop);

    REVLibError configureError = m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, Constants.kPersistMode);
    if (configureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure drive motor", new IOException());
    }

    m_leftPIDController = m_leftMotor.getClosedLoopController();

    m_rightMotor = new SparkMax(ChamberConstants.kRightMotorID, MotorType.kBrushless);
    
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.inverted(ChamberConstants.kRightMotorReversed)
    .smartCurrentLimit(ChamberConstants.kCurrentLimit)
    .idleMode(IdleMode.kBrake);
    rightMotorConfig.encoder
    .velocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);
    rightMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    ChamberConstants.kRightPIDF.controllerSet(rightMotorConfig.closedLoop);

    REVLibError rightConfigureError = m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, Constants.kPersistMode);
    if (rightConfigureError != REVLibError.kOk) {
      throw new UncheckedIOException("Failed to configure drive motor", new IOException());
    }
    
    m_rightPIDController = m_rightMotor.getClosedLoopController();

    m_noteDetectedSensor = new DigitalInput(ChamberConstants.kSensorID);

    m_intake = intake;
    m_shooter = shooter;
  }

  public void initializePreloaded() {
    m_state = m_chooser.getSelected();
    if (m_state == State.CHAMBERED) {
      m_shooter.idleShooter();
    }
  }

  private void setSpeed(double speed) {
    m_leftPIDController.setReference(speed, ControlType.kVelocity);
    m_rightPIDController.setReference(speed, ControlType.kVelocity);
  }

  private void stopChamber() {
    if (m_state != State.EMPTY) {
      m_leftMotor.stopMotor();
      m_rightMotor.stopMotor();
      m_state = State.EMPTY;
    }
  }

  private boolean isNoteDetected() {
    return m_noteDetectedSensor.get() == ChamberConstants.kSensorNoteDetectedValue;
  }

  public boolean isNoteChambered() {
    return m_state == State.CHAMBERED;
  }

  public boolean chamberNote() {
    switch (m_state) {
      case EMPTY: {
        double speed = m_intakingSpeed.get();
        setSpeed(speed);
        m_intake.startIntake();
        m_state = State.CHAMBERING;
        return false;
      }
      default: return true;
    }
  }

  public boolean cancelChambering() {
    switch (m_state) {
      case CHAMBERING: {
        stopChamber();
        m_intake.stopIntake();
        return false;
      }
      default: return true;
    }
  }

  public boolean shootNote(double speed) {
    switch (m_state) {
      case CHAMBERED: {
        m_shooter.revShooter(speed);
        m_state = State.SHOOTING;
        return false;
      }
      default: return true;
    }
  }

  public boolean ejectNote() {
    switch (m_state) {
      case CHAMBERED:
      case SHOOTING: {
        m_shooter.revShooter(ShooterConstants.kIdleSpeed);
        m_state = State.EJECTING;
        return false;
      }
      default: return true;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Chamber State", m_state.name()); 
    switch (m_state) {
      case EMPTY: {
        break;
      }
      case CHAMBERING: {
        if (isNoteDetected()) {
          m_intake.stopIntake();
          setSpeed(ChamberConstants.kAligningSpeed);
          m_state = State.ALIGNING;
        }
        break;
      }
      case ALIGNING: {
        if (!isNoteDetected()) {
          stopChamber();
          m_shooter.idleShooter();
          m_state = State.CHAMBERED;
        }
        break;
      }
      case CHAMBERED: {
        break;
      }
      case SHOOTING: {
        if (m_shooter.isReadyToShoot()) {
          m_shootTimeSeconds = RobotController.getFPGATime() / 1000000.0;
          setSpeed(ChamberConstants.kShootingSpeed);
          m_state = State.CLEARING;
        }
        break;
      }
      case EJECTING: {
        m_shootTimeSeconds = RobotController.getFPGATime() / 1000000.0;
        setSpeed(ChamberConstants.kShootingSpeed);
        m_state = State.CLEARING;
        break;
      }
      case CLEARING: {
        double currentTimeSeconds = RobotController.getFPGATime() / 1000000.0;
        if (m_shootTimeSeconds + ChamberConstants.kClearingTimeSeconds <= currentTimeSeconds) {
          stopChamber();
          m_shooter.stopShooter();
        }
        break;
      }
    }
  }
}
