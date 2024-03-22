// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utilities.TunableDouble;
import frc.robot.utilities.Utilities;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_topMotor;
  private final CANSparkMax m_bottomMotor;
  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_bottomEncoder;
  private final SparkPIDController m_topPIDController;
  private final SparkPIDController m_bottomPIDController;

  private double m_idealSpeed = 0.0;

  private final TunableDouble m_speed =
    new TunableDouble("Intake.speed", IntakeConstants.kSpeed);

  public Intake() {
    m_topMotor = new CANSparkMax(IntakeConstants.kTopMotorID, MotorType.kBrushless);
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(IntakeConstants.kTopMotorReversed);
    m_topMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_topEncoder = m_topMotor.getEncoder();
    m_topEncoder.setVelocityConversionFactor(IntakeConstants.kTopVelocityConversionFactor);
    m_topPIDController = m_topMotor.getPIDController();
    m_topPIDController.setFeedbackDevice(m_topEncoder);
    IntakeConstants.kTopPIDF.controllerSet(m_topPIDController);
    Utilities.burnMotor(m_topMotor);

    m_bottomMotor = new CANSparkMax(IntakeConstants.kBottomMotorID, MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotor.setInverted(IntakeConstants.kBottomMotorReversed);
    m_bottomMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_bottomEncoder = m_bottomMotor.getEncoder();
    m_bottomEncoder.setVelocityConversionFactor(IntakeConstants.kBottomVelocityConversionFactor);
    m_bottomPIDController = m_bottomMotor.getPIDController();
    m_bottomPIDController.setFeedbackDevice(m_bottomEncoder);
    IntakeConstants.kBottomPIDF.controllerSet(m_bottomPIDController);
    Utilities.burnMotor(m_bottomMotor);
  }

  private void setSpeed(double speed) {
    m_idealSpeed = speed;
    m_topPIDController.setReference(m_idealSpeed, ControlType.kVelocity);
    m_bottomPIDController.setReference(m_idealSpeed, ControlType.kVelocity);
  }

  public void startIntake() {
    setSpeed(m_speed.get());
  }
  
  public void stopIntake() {
    m_idealSpeed = 0.0;
    m_topMotor.stopMotor();
    m_bottomMotor.stopMotor();
  }

  public void reverseIntake() {
    setSpeed(-m_speed.get());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Running", (m_idealSpeed != 0.0));
  }
}
