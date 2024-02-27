// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.TunableConstant;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;

  private final TunableConstant m_idleSpeed =
    new TunableConstant("Shooter.idleSpeed", ShooterConstants.kIdleSpeed);

  private double m_idealLeftSpeed;
  private double m_idealRightSpeed;

  private enum State {
    STOPPED,
    IDLING,
    REVVING
  }
  private State m_state = State.STOPPED;

  public Shooter() {
    m_leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(ShooterConstants.kLeftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ShooterConstants.kVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(ShooterConstants.kPositionConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ShooterConstants.kPID.p(), 0);
    m_leftPIDController.setI(ShooterConstants.kPID.i(), 0);
    m_leftPIDController.setD(ShooterConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(ShooterConstants.kRightMotorReversed);
    m_rightMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

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

  private void setSpeeds(double leftSpeed, double rightSpeed) {
    m_idealLeftSpeed = leftSpeed;
    m_idealRightSpeed = rightSpeed;
    m_rightPIDController.setReference(m_idealRightSpeed, ControlType.kVelocity);
    m_leftPIDController.setReference(m_idealLeftSpeed, ControlType.kVelocity);
  }

  public void stopShooter() {
    m_idealLeftSpeed = 0.0;
    m_idealRightSpeed = 0.0;
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    m_state = State.STOPPED;
  }

  public void idleShooter() {
    double idleSpeed = m_idleSpeed.get();
    setSpeeds(idleSpeed, idleSpeed);
    m_state = State.IDLING;
  }

  public void revShooter(double leftSpeed, double rightSpeed) {
    setSpeeds(leftSpeed, rightSpeed);
    m_state = State.REVVING;
  }

  public boolean isReadyToShoot() {
    switch (m_state) {
      case REVVING: {
        double rightMotorVelocity = m_rightEncoder.getVelocity();
        double leftMotorVelocity = m_leftEncoder.getVelocity();
        return (
          rightMotorVelocity >= m_idealRightSpeed * (1.0 - ShooterConstants.kShootingTolerance) &&
          rightMotorVelocity <= m_idealRightSpeed * (1.0 + ShooterConstants.kShootingTolerance) &&
          leftMotorVelocity >= m_idealLeftSpeed * (1.0 - ShooterConstants.kShootingTolerance) &&
          leftMotorVelocity <= m_idealLeftSpeed * (1.0 + ShooterConstants.kShootingTolerance)
        );
      }
      default: return false;
    }
  }

  @Override
  public void periodic() {}
}
