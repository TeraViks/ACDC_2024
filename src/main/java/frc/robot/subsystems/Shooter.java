// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  SparkPIDController m_leftPIDController;
  SparkPIDController m_rightPIDController;

  double m_idealLeftSpeed;
  double m_idealRightSpeed;

  public Shooter(int leftMotorID, int rightMotorID, boolean leftMotorReversed, boolean rightMotorReversed) {
    m_leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(leftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ShooterConstants.kVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(ShooterConstants.kPositionConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ShooterConstants.kPID.p(), 0);
    m_leftPIDController.setI(ShooterConstants.kPID.i(), 0);
    m_leftPIDController.setD(ShooterConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(rightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ShooterConstants.kVelocityConversionFactor);
    m_rightEncoder.setPositionConversionFactor(ShooterConstants.kPositionConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(ShooterConstants.kPID.p(), 0);
    m_rightPIDController.setI(ShooterConstants.kPID.i(), 0);
    m_rightPIDController.setD(ShooterConstants.kPID.d(), 0);
    m_rightPIDController.setFF(0);
  }

  public void shoot(double rightMotorSpeed, double leftMotorSpeed) {
    m_idealLeftSpeed = leftMotorSpeed;
    m_idealRightSpeed = rightMotorSpeed;
  }

  public void stopShooter() {
    m_idealLeftSpeed = 0.0;
    m_idealRightSpeed = 0.0;
  }

  public boolean isReadyToShoot() {
    double rightMotorVelocity = m_rightEncoder.getVelocity();
    double leftMotorVelocity = m_leftEncoder.getVelocity();
    return (
      rightMotorVelocity >= m_idealRightSpeed * (1.0 - ShooterConstants.kShootingTolerance) &&
      rightMotorVelocity <= m_idealRightSpeed * (1.0 + ShooterConstants.kShootingTolerance) &&
      leftMotorVelocity >= m_idealLeftSpeed * (1.0 - ShooterConstants.kShootingTolerance) &&
      leftMotorVelocity <= m_idealLeftSpeed * (1.0 + ShooterConstants.kShootingTolerance)
    );
  }

  @Override
  public void periodic() {
    m_rightPIDController.setReference(m_idealRightSpeed, ControlType.kVelocity);
    m_leftPIDController.setReference(m_idealLeftSpeed, ControlType.kVelocity);
  }
}
