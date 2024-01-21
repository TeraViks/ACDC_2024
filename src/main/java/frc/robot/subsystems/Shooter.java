// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  SparkPIDController m_leftPIDController;
  SparkPIDController m_rightPIDController;

  double m_currentLeftSpeed;
  double m_currentRightSpeed;

  double m_kLeftP;
  double m_kLeftI;
  double m_kLeftD;
  double m_kRightP;
  double m_kRightI;
  double m_kRightD;

  double m_kShootingTolerance = 10.0;

  /** Creates a new Shooter. */
  public Shooter(int leftMotorID, int rightMotorID, boolean leftMotorReversed, boolean rightMotorReversed) {
    m_leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(leftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ShooterConstants.kFlywheelEncoderDistancePerPulse * 10.0);
    m_leftEncoder.setPositionConversionFactor(ShooterConstants.kFlywheelEncoderDistancePerPulse);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(m_kLeftP, 0);
    m_leftPIDController.setI(m_kLeftI, 0);
    m_leftPIDController.setD(m_kLeftD, 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(rightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);

    m_rightEncoder = m_leftMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ShooterConstants.kFlywheelEncoderDistancePerPulse * 10.0);
    m_rightEncoder.setPositionConversionFactor(ShooterConstants.kFlywheelEncoderDistancePerPulse);

    m_rightPIDController = m_leftMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(m_kRightD, 0);
    m_rightPIDController.setI(m_kRightI, 0);
    m_rightPIDController.setD(m_kRightD, 0);
    m_rightPIDController.setFF(0);
  }

  public void shoot(double rightMotorSpeed, double leftMotorSpeed) {
    m_currentLeftSpeed = leftMotorSpeed;
    m_currentRightSpeed = rightMotorSpeed;
  }

  public void stopShooter() {
    m_currentLeftSpeed = 0.0;
    m_currentRightSpeed = 0.0;
  }

  public boolean isReadyToShoot() {
    return (
      m_rightMotor.get() <= m_currentRightSpeed + m_kShootingTolerance || 
      m_rightMotor.get() >= m_currentRightSpeed - m_kShootingTolerance &&
      m_leftMotor.get() <= m_currentLeftSpeed + m_kShootingTolerance || 
      m_leftMotor.get() >= m_currentLeftSpeed - m_kShootingTolerance
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_rightPIDController.setReference(m_currentRightSpeed, ControlType.kVelocity);
    m_leftPIDController.setReference(m_currentLeftSpeed, ControlType.kVelocity);
  }
}
