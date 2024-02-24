// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotorTop;
  private final CANSparkMax m_intakeMotorBottom;
  private final RelativeEncoder m_intakeEncoderTop;
  private final RelativeEncoder m_intakeEncoderBottom;
  private final SparkPIDController m_intakePidControllerTop;
  private final SparkPIDController m_intakePidControllerBottom;

  public Intake(int topMotorID, int bottomMotorID, boolean motorReversed) {
    m_intakeMotorTop = new CANSparkMax(topMotorID, MotorType.kBrushless);
    m_intakeMotorTop.restoreFactoryDefaults();
    m_intakeMotorTop.setInverted(motorReversed);
    m_intakeMotorTop.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
    m_intakeEncoderTop = m_intakeMotorTop.getEncoder();
    m_intakeEncoderTop.setVelocityConversionFactor(IntakeConstants.kTopVelocityConversionFactor);
    m_intakePidControllerTop = m_intakeMotorTop.getPIDController();
    m_intakePidControllerTop.setFeedbackDevice(m_intakeEncoderTop);
    m_intakePidControllerTop.setP(IntakeConstants.kIntakePIDTop.p(), 0);
    m_intakePidControllerTop.setI(IntakeConstants.kIntakePIDTop.i(), 0);
    m_intakePidControllerTop.setD(IntakeConstants.kIntakePIDTop.d(), 0);
    m_intakePidControllerTop.setFF(0);
   
    m_intakeMotorBottom = new CANSparkMax(bottomMotorID, MotorType.kBrushless);
    m_intakeMotorBottom.restoreFactoryDefaults();
    m_intakeMotorBottom.setInverted(!motorReversed);
    m_intakeMotorBottom.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
    m_intakeEncoderBottom = m_intakeMotorBottom.getEncoder();
    m_intakeEncoderBottom.setVelocityConversionFactor(IntakeConstants.kBottomVelocityConversionFactor);
    m_intakePidControllerBottom = m_intakeMotorBottom.getPIDController();
    m_intakePidControllerBottom.setFeedbackDevice(m_intakeEncoderBottom);
    m_intakePidControllerBottom.setP(IntakeConstants.kIntakePIDBottom.p(), 0);
    m_intakePidControllerBottom.setI(IntakeConstants.kIntakePIDBottom.i(), 0);
    m_intakePidControllerBottom.setD(IntakeConstants.kIntakePIDBottom.d(), 0);
    m_intakePidControllerBottom.setFF(0);
  }
  //speed is in m/s
  public void startIntake(double speed) {
    m_intakePidControllerTop.setReference(speed, ControlType.kVelocity);
    m_intakePidControllerBottom.setReference(speed, ControlType.kVelocity);
  }
  
  public void stopIntake() {
    m_intakeMotorTop.stopMotor();
    m_intakeMotorBottom.stopMotor();
  }

  public double getTopIntakeSpeed() {
    return m_intakeEncoderTop.getVelocity();
  }

  public double getBottomIntakeSpeed() {
  return m_intakeEncoderBottom.getVelocity();
  }

  @Override
  public void periodic() {}
}
