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
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPIDController;
  private final SparkPIDController m_rightPIDController;

  public Climber(int climberRightMotorID, int climberLeftMotorID, boolean motorReversed) {
    m_leftMotor = new CANSparkMax(climberLeftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setInverted(motorReversed);
    m_leftMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setVelocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

    m_leftPIDController = m_leftMotor.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_leftPIDController.setP(ClimberConstants.kPID.p(), 0);
    m_leftPIDController.setI(ClimberConstants.kPID.i(), 0);
    m_leftPIDController.setD(ClimberConstants.kPID.d(), 0);
    m_leftPIDController.setFF(0);

    m_rightMotor = new CANSparkMax(climberRightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    m_rightMotor.setInverted(motorReversed);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setVelocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

    m_rightPIDController = m_rightMotor.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);
    m_rightPIDController.setP(ClimberConstants.kPID.p(), 0);
    m_rightPIDController.setI(ClimberConstants.kPID.i(), 0);
    m_rightPIDController.setD(ClimberConstants.kPID.d(), 0);
    m_rightPIDController.setFF(0);
  }

  public void startClimber(double speed) {
    m_leftPIDController.setReference(speed, ControlType.kVelocity);
    m_rightPIDController.setReference(speed, ControlType.kVelocity);
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
