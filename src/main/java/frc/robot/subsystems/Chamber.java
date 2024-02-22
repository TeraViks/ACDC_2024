package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChamberConstants; 
import frc.robot.PID;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Chamber extends SubsystemBase{
  private final CANSparkMax m_leftChamberMotor;
  private final CANSparkMax m_rightChamberMotor;
  private final RelativeEncoder m_leftChamberEncoder;
  private final RelativeEncoder m_rightChamberEncoder;
  private final SparkPIDController m_leftChamberPIDController;
  private final SparkPIDController m_rightChamberPIDController;
  
  private final PID m_leftChamberMotorPID = ChamberConstants.kLeftPID;
  private final PID m_rightChamberMotorPID = ChamberConstants.kRightPID;

  private final DigitalInput m_noteChamberedSensor;

  public Chamber(int leftChamberMotorID, int rightChamberMotorID, boolean motorReversed, int sensorID) {
    m_leftChamberMotor = new CANSparkMax(leftChamberMotorID, MotorType.kBrushless);
    m_leftChamberMotor.restoreFactoryDefaults();
    m_leftChamberMotor.setInverted(motorReversed);
    m_leftChamberMotor.setSmartCurrentLimit(ChamberConstants.kChamberCurrentLimit);

    m_leftChamberEncoder = m_leftChamberMotor.getEncoder();
    m_leftChamberEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_leftChamberPIDController = m_leftChamberMotor.getPIDController();
    m_leftChamberPIDController.setFeedbackDevice(m_leftChamberEncoder);
    m_leftChamberPIDController.setP(m_leftChamberMotorPID.p(), 0);
    m_leftChamberPIDController.setI(m_leftChamberMotorPID.i(), 0);
    m_leftChamberPIDController.setD(m_leftChamberMotorPID.d(), 0);
    m_leftChamberPIDController.setFF(0);

    m_rightChamberMotor = new CANSparkMax(rightChamberMotorID, MotorType.kBrushless);
    m_rightChamberMotor.restoreFactoryDefaults();
    m_rightChamberMotor.setInverted(!motorReversed);
    m_rightChamberMotor.setSmartCurrentLimit(ChamberConstants.kChamberCurrentLimit);

    m_rightChamberEncoder = m_rightChamberMotor.getEncoder();
    m_rightChamberEncoder.setVelocityConversionFactor(ChamberConstants.kChamberVelocityConversionFactor);

    m_rightChamberPIDController = m_rightChamberMotor.getPIDController();
    m_rightChamberPIDController.setFeedbackDevice(m_rightChamberEncoder);
    m_rightChamberPIDController.setP(m_rightChamberMotorPID.p(), 0);
    m_rightChamberPIDController.setI(m_rightChamberMotorPID.i(), 0);
    m_rightChamberPIDController.setD(m_rightChamberMotorPID.d(), 0);
    m_rightChamberPIDController.setFF(0);

    m_noteChamberedSensor = new DigitalInput(sensorID);
  }
  
  public boolean isNoteChambered() {
    return !m_noteChamberedSensor.get();
  }

  // speed in m/s
  public void moveNote(double speed) {
    m_leftChamberPIDController.setReference(speed, ControlType.kVelocity);
    m_rightChamberPIDController.setReference(speed, ControlType.kVelocity);
  }

  public double getLeftIntakeSpeed() {
    return m_leftChamberEncoder.getVelocity();
  }

  public double getRightIntakeSpeed() {
    return m_rightChamberEncoder.getVelocity();
  }
}
