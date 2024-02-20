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
import frc.robot.PID;
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

  PID m_leftMotorPID = ShooterConstants.kLeftPID;
  PID m_rightMotorPID = ShooterConstants.kLeftPID;

  double m_kShootingTolerance = 0.05;

  public Shooter(int leftMotorID, int rightMotorID, boolean leftMotorReversed, boolean rightMotorReversed) {
    m_leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(leftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ShooterConstants.kShooterVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(ShooterConstants.kShooterPositionConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(m_leftMotorPID.p(), 0);
    m_leftPIDController.setI(m_leftMotorPID.i(), 0);
    m_leftPIDController.setD(m_leftMotorPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(rightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

    m_rightEncoder = m_leftMotor.getEncoder();
     m_rightEncoder.setVelocityConversionFactor(ShooterConstants.kShooterVelocityConversionFactor);
    m_rightEncoder.setPositionConversionFactor(ShooterConstants.kShooterPositionConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(m_rightMotorPID.p(), 0);
    m_rightPIDController.setI(m_rightMotorPID.i(), 0);
    m_rightPIDController.setD(m_rightMotorPID.d(), 0);
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

    double rightTolerance = m_idealRightSpeed * m_kShootingTolerance;
    double leftTolerance = m_idealRightSpeed * m_kShootingTolerance;
    return (
      rightMotorVelocity * (1.0 - rightTolerance) <= m_idealRightSpeed || 
      rightMotorVelocity * (1.0 + rightTolerance) >= m_idealRightSpeed &&
      leftMotorVelocity  * (1.0 -leftTolerance) <= m_idealLeftSpeed || 
      leftMotorVelocity  * (1.0 + leftTolerance) >= m_idealLeftSpeed
    );
  }

  @Override
  public void periodic() {
    m_rightPIDController.setReference(m_idealRightSpeed, ControlType.kVelocity);
    m_leftPIDController.setReference(m_idealLeftSpeed, ControlType.kVelocity);
  }
}
