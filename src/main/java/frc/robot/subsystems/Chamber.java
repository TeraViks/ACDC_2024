package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChamberConstants;

public class Chamber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;

  private final DigitalInput m_noteChamberedSensor;

  public Chamber(int leftChamberMotorID, int rightChamberMotorID, boolean motorReversed, int sensorID) {
    m_leftMotor = new CANSparkMax(leftChamberMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(motorReversed);
    m_leftMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ChamberConstants.kPID.p(), 0);
    m_leftPIDController.setI(ChamberConstants.kPID.i(), 0);
    m_leftPIDController.setD(ChamberConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(rightChamberMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(!motorReversed);
    m_rightMotor.setSmartCurrentLimit(ChamberConstants.kCurrentLimit);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(ChamberConstants.kPID.p(), 0);
    m_rightPIDController.setI(ChamberConstants.kPID.i(), 0);
    m_rightPIDController.setD(ChamberConstants.kPID.d(), 0);
    m_rightPIDController.setFF(0);

    m_noteChamberedSensor = new DigitalInput(sensorID);
  }

  public boolean isNoteChambered() {
    return !m_noteChamberedSensor.get();
  }

  // speed in m/s
  public void moveNote(double speed) {
    m_leftPIDController.setReference(speed, ControlType.kVelocity);
    m_rightPIDController.setReference(speed, ControlType.kVelocity);
  }

  public double getLeftIntakeSpeed() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightIntakeSpeed() {
    return m_rightEncoder.getVelocity();
  }
}
