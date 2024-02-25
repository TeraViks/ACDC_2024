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
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
private final CANSparkMax m_climberLeftMotor;
private final CANSparkMax m_climberRightMotor;
private final RelativeEncoder m_leftClimberEncoder;
private final RelativeEncoder m_rightClimberEncoder;
private final SparkPIDController m_climberPidControllerLeft;
private final SparkPIDController m_climberPidControllerRight;
public static final double kClimberVelocityConversionFactor = (0.2);

public ClimberSubsystem(int climberRightMotorID, int climberLeftMotorID, boolean motorReversed) {
  m_climberLeftMotor = new CANSparkMax(climberLeftMotorID, MotorType.kBrushless);
  m_climberLeftMotor.setInverted(motorReversed);
  m_leftClimberEncoder = m_climberLeftMotor.getEncoder();
  m_climberPidControllerLeft = m_climberLeftMotor.getPIDController();
  m_climberPidControllerLeft.setFeedbackDevice(m_leftClimberEncoder);
  m_climberPidControllerLeft.setP(ClimberConstants.kClimberPIDLeft.p(), 0);
  m_climberPidControllerLeft.setI(ClimberConstants.kClimberPIDLeft.i(), 0);
  m_climberPidControllerLeft.setD(ClimberConstants.kClimberPIDLeft.d(), 0);
  m_climberPidControllerLeft.setFF(0);
 
   
  m_climberRightMotor = new CANSparkMax(climberRightMotorID, MotorType.kBrushless);
  m_climberRightMotor.setInverted(motorReversed);
  m_rightClimberEncoder = m_climberRightMotor.getEncoder();
  m_climberPidControllerRight = m_climberRightMotor.getPIDController();
  m_climberPidControllerRight.setFeedbackDevice(m_rightClimberEncoder);
  m_climberPidControllerRight.setP(ClimberConstants.kClimberPIDRight.p(), 0);
  m_climberPidControllerRight.setI(ClimberConstants.kClimberPIDRight.i(), 0);
  m_climberPidControllerRight.setD(ClimberConstants.kClimberPIDRight.d(), 0);
  m_climberPidControllerRight.setFF(0);
  }
  public void startClimber(double speed) {
    m_climberPidControllerLeft.setReference(speed, ControlType.kVelocity);
    m_climberPidControllerRight.setReference(speed, ControlType.kVelocity);
  }
  
  public void stopClimber() {
  m_climberLeftMotor.stopMotor();
  m_climberRightMotor.stopMotor();
  }

  public double getLeftClimberSpeed() {
    return m_leftClimberEncoder.getVelocity();
  }

  public double getRightClimberSpeed() {
  return m_rightClimberEncoder.getVelocity();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
