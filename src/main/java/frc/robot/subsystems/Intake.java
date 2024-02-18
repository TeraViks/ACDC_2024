// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Intake extends SubsystemBase {
  CANSparkMax m_intakeMotor1;
  CANSparkMax m_intakeMotor2;
  Ultrasonic m_ultrasonicSensor;

  public Intake(int motorID, boolean motorReversed, int pingPort, int echoPort) {
    m_intakeMotor1 = new CANSparkMax(motorID, MotorType.kBrushless);
    m_intakeMotor1.restoreFactoryDefaults();
    m_intakeMotor1.setInverted(motorReversed);
    m_intakeMotor1.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);

    m_intakeMotor2 = new CANSparkMax(motorID, MotorType.kBrushless);
    m_intakeMotor2.restoreFactoryDefaults();
    m_intakeMotor2.setInverted(motorReversed);
    m_intakeMotor2.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);


    m_ultrasonicSensor = new Ultrasonic(pingPort, echoPort);
  }

  public void startIntake(double speed) {
    m_intakeMotor1.set(speed);
    m_intakeMotor2.set(speed);
  }

  public void stopIntake() {
    m_intakeMotor1.stopMotor();
    m_intakeMotor2.stopMotor();
  }

  public boolean isNoteChambered() {
    return(m_ultrasonicSensor.getRangeInches() <= 1.0);
  }

  public double intakeSpeed() {
    return m_intakeMotor1.get();
    return m_intakeMotor2.get();
  }

  @Override
  public void periodic() {}
}
