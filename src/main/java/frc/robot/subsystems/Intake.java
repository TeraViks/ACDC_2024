// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Intake extends SubsystemBase {
  CANSparkMax m_intakeMotor;
  Ultrasonic m_ultrasonicSensor;
  /** Creates a new Intake. */
  public Intake(int motorID, boolean motorReversed, int pingPort, int echoPort) {
    m_intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setInverted(motorReversed);
    m_intakeMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);

    m_ultrasonicSensor = new Ultrasonic(pingPort, echoPort);
  }

  public void startIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stopIntake() {
    m_intakeMotor.stopMotor();
  }

  public boolean isNoteChambered() {
    return(m_ultrasonicSensor.getRangeInches() <= 1.0);
  }

  public double intakeSpeed() {
    return m_intakeMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
