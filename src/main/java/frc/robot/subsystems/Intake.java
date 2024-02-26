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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_topMotor;
  private final CANSparkMax m_bottomMotor;
  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_bottomEncoder;
  private final SparkPIDController m_topPIDController;
  private final SparkPIDController m_bottomPIDController;

  public Intake(int topMotorID, int bottomMotorID, boolean motorReversed) {
    m_topMotor = new CANSparkMax(topMotorID, MotorType.kBrushless);
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(motorReversed);
    m_topMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_topEncoder = m_topMotor.getEncoder();
    m_topEncoder.setVelocityConversionFactor(IntakeConstants.kTopVelocityConversionFactor);
    m_topPIDController = m_topMotor.getPIDController();
    m_topPIDController.setFeedbackDevice(m_topEncoder);
    m_topPIDController.setP(IntakeConstants.kTopPID.p(), 0);
    m_topPIDController.setI(IntakeConstants.kTopPID.i(), 0);
    m_topPIDController.setD(IntakeConstants.kTopPID.d(), 0);
    m_topPIDController.setFF(0);
   
    m_bottomMotor = new CANSparkMax(bottomMotorID, MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotor.setInverted(!motorReversed);
    m_bottomMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_bottomEncoder = m_bottomMotor.getEncoder();
    m_bottomEncoder.setVelocityConversionFactor(IntakeConstants.kBottomVelocityConversionFactor);
    m_bottomPIDController = m_bottomMotor.getPIDController();
    m_bottomPIDController.setFeedbackDevice(m_bottomEncoder);
    m_bottomPIDController.setP(IntakeConstants.kIntakePIDBottom.p(), 0);
    m_bottomPIDController.setI(IntakeConstants.kIntakePIDBottom.i(), 0);
    m_bottomPIDController.setD(IntakeConstants.kIntakePIDBottom.d(), 0);
    m_bottomPIDController.setFF(0);
  }

  // speed is in m/s
  public void startIntake(double speed) {
    m_topPIDController.setReference(speed, ControlType.kVelocity);
    m_bottomPIDController.setReference(speed, ControlType.kVelocity);
  }
  
  public void stopIntake() {
    m_topMotor.stopMotor();
    m_bottomMotor.stopMotor();
  }

  public double getTopIntakeSpeed() {
    return m_topEncoder.getVelocity();
  }

  public double getBottomIntakeSpeed() {
    return m_bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {}
}
