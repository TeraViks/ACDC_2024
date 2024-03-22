package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChamberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.TunableDouble;
import frc.robot.utilities.Utilities;

public class Chamber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;
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

    m_leftMotor = new CANSparkMax(ChamberConstants.kLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(ChamberConstants.kLeftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    ChamberConstants.kLeftPIDF.controllerSet(m_leftPIDController);
    Utilities.burnMotor(m_leftMotor);

    m_rightMotor = new CANSparkMax(ChamberConstants.kRightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(ChamberConstants.kRightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    ChamberConstants.kRightPIDF.controllerSet(m_rightPIDController);
    Utilities.burnMotor(m_rightMotor);

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
