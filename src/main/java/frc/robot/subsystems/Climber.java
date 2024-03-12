// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utilities;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;
  private final SparkLimitSwitch m_leftLimitSwitch;
  private final SparkLimitSwitch m_rightLimitSwith;
  private double m_idealLeftSpeed;
  private double m_idealRightSpeed;

  private enum State {
    UNINITIALIZED,
    INITIALIZED,
  }

  private State m_state;

  public Climber() {
    m_leftMotor = new CANSparkMax(ClimberConstants.kLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(ClimberConstants.kLeftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ClimberConstants.kVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(ClimberConstants.kPositionConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ClimberConstants.kPID.p(), 0);
    m_leftPIDController.setI(ClimberConstants.kPID.i(), 0);
    m_leftPIDController.setD(ClimberConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_leftLimitSwitch = m_leftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    Utilities.burnMotor(m_leftMotor);

    m_rightMotor = new CANSparkMax(ClimberConstants.kRightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    m_rightMotor.setInverted(ClimberConstants.kRightMotorReversed);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_rightEncoder = m_rightMotor.getEncoder();

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(ClimberConstants.kPID.p(), 0);
    m_rightPIDController.setI(ClimberConstants.kPID.i(), 0);
    m_rightPIDController.setD(ClimberConstants.kPID.d(), 0);
    m_rightPIDController.setFF(0);

    m_rightLimitSwith = m_rightMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    Utilities.burnMotor(m_rightMotor);
    m_state = State.UNINITIALIZED;
    initialize();
  }

  private void initialize() {
    m_leftPIDController.setReference(ClimberConstants.kInitializingSpeed, ControlType.kVelocity);
    m_rightPIDController.setReference(ClimberConstants.kInitializingSpeed, ControlType.kVelocity);
  }

  private double transformSpeed(double position, double speed) {
    if (position >= ClimberConstants.kMaxExtendedLength - ClimberConstants.kConstrainedRange &&
        speed > 0.0) {
      return speed * ((ClimberConstants.kMaxExtendedLength - position) / 
        ClimberConstants.kConstrainedRange);
    } else if (
        position <= ClimberConstants.kMinRetractedLength + ClimberConstants.kConstrainedRange &&
        speed < 0.0) {
      return speed * ((position - ClimberConstants.kMinRetractedLength) /
        ClimberConstants.kConstrainedRange);
    } else {
      return speed;
    }
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    m_idealLeftSpeed = leftSpeed;
    m_idealRightSpeed = rightSpeed;
  }

  public void stopClimber() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case UNINITIALIZED: {
        if (m_leftLimitSwitch.isPressed() && m_rightLimitSwith.isPressed()) {
          m_state = State.INITIALIZED;
          break;
        }
      }
      case INITIALIZED: {
        double leftPosition = m_leftEncoder.getPosition();
        double rightPosition = m_rightEncoder.getPosition();
        double leftSpeed = (m_leftMotor.getLastError() == REVLibError.kOk) ? m_idealLeftSpeed : 0.0;
        double rightSpeed = (m_rightMotor.getLastError() == REVLibError.kOk) ? m_idealRightSpeed : 0.0;
        m_leftPIDController.setReference(transformSpeed(leftPosition, leftSpeed), ControlType.kVelocity);
        m_rightPIDController.setReference(transformSpeed(rightPosition, rightSpeed), ControlType.kVelocity);
        break;
      }
    }
  }
}
