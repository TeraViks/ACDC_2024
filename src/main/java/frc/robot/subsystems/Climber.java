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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utilities;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;

  public Climber() {
    m_leftMotor = new CANSparkMax(ClimberConstants.kLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(ClimberConstants.kLeftMotorReversed);
    m_leftMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftMotor.getEncoder();

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ClimberConstants.kPID.p(), 0);
    m_leftPIDController.setI(ClimberConstants.kPID.i(), 0);
    m_leftPIDController.setD(ClimberConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

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

    Utilities.burnMotor(m_rightMotor);
  }

  public void startClimber(double leftSpeed, double rightSpeed) {
    m_leftPIDController.setReference(leftSpeed, ControlType.kDutyCycle);
    m_rightPIDController.setReference(rightSpeed, ControlType.kDutyCycle);
  }

  public void stopClimber() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  public double getLeftClimberSpeed() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightClimberSpeed() {
    return m_rightEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
