package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChamberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.TunableConstant;

public class Chamber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;
  private final Intake m_intake;
  private final Shooter m_shooter;

  private final DigitalInput m_noteChamberedSensor;

  private final TunableConstant m_speed = new TunableConstant("Chamber.speed", ChamberConstants.kChamberingSpeed);

  private enum State {
    STOPPED,
    CHAMBERING,
    SHOOTING,
    CLEARING
  }
  private State m_state = State.STOPPED;
  private double m_shootTimeSeconds = 0.0;

  public Chamber(Intake intake, Shooter shooter) {
    m_leftMotor = new CANSparkMax(ChamberConstants.kLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(ChamberConstants.kLeftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ChamberConstants.kLeftPID.p(), 0);
    m_leftPIDController.setI(ChamberConstants.kLeftPID.i(), 0);
    m_leftPIDController.setD(ChamberConstants.kLeftPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(ChamberConstants.kRightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(ChamberConstants.kRightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(ChamberConstants.kRightPID.p(), 0);
    m_rightPIDController.setI(ChamberConstants.kRightPID.i(), 0);
    m_rightPIDController.setD(ChamberConstants.kRightPID.d(), 0);
    m_rightPIDController.setFF(0);

    m_noteChamberedSensor = new DigitalInput(ChamberConstants.kSensorID);

    m_intake = intake;
    m_shooter = shooter;
  }

  public boolean isNoteChambered() {
    return !m_noteChamberedSensor.get();
  }

  private void setSpeed(double speed) {
    m_leftPIDController.setReference(speed, ControlType.kVelocity);
    m_rightPIDController.setReference(speed, ControlType.kVelocity);
  }

  public void chamberNote() {
    if (!isNoteChambered()) {
      double speed = m_speed.get();
      setSpeed(speed);
      m_state = State.CHAMBERING;
    }
  }

  public boolean shootNote() { // XXX Pass in interpolated speeds.
    if (isNoteChambered()) {return true;}
    double shooterSpeed = ShooterConstants.kIdleSpeed; // XXX Use interpolated speeds.
    m_shooter.revShooter(shooterSpeed, shooterSpeed);
    m_state = State.SHOOTING;
    return false;
  }

  public void stopChamber() {
    if (m_state != State.STOPPED) {
      m_leftMotor.stopMotor();
      m_rightMotor.stopMotor();
      m_state = State.STOPPED;
    }
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case STOPPED: {}
      case CHAMBERING: {
        if (isNoteChambered()) {
          stopChamber();
          m_intake.stopIntake();
        }
      }
      case SHOOTING: {
        if (m_shooter.isReadyToShoot()) {
          m_shootTimeSeconds = RobotController.getFPGATime();
          setSpeed(ChamberConstants.kShootingSpeed);
          m_state = State.CLEARING;
        }
      }
      case CLEARING: {
        double currentTimeSeconds = RobotController.getFPGATime();
        if (m_shootTimeSeconds + ChamberConstants.kRecoilTimeSeconds <= currentTimeSeconds) {
          stopChamber();
          m_shooter.idleShooter();
        }
      }
    }
  }
}
