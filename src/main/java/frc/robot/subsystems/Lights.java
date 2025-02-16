// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//This file is supposed to identify a color and flash it when prompted.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends SubsystemBase {
  private Spark blinkin;

  /** Creates a new Lights. */
  public Lights() {
    blinkin = new Spark(0); // Assuming the Blinkin is connected to PWM port 0
  }
  public enum Color {
    RED, 
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
    PURPLE,
    WHITE,
    PINK,
    OFF
  }

  @Override
  public void periodic() {
  }
  private double getColor(Color color) {
    switch (color) {
      case RED: return 0.61;
      case ORANGE: return 0.63; 
      case YELLOW: return 0.69; // Hard on the eyes
      case GREEN: return 0.77;
      case BLUE: return 0.87; 
      case PURPLE: return 0.89;   
      case WHITE: return 0.93; // Also very hard on the eyes, not as bad as yellow
      case PINK: return 0.57; // Worse on the eyes than white, but not as bad as yellow
      case OFF: return 0.99;
      default: return 0.99;
    }
  } /* getColor sets values for colors found in Color. 
  More color values can be found at https://docs.revrobotics.com/sites-test-ion/rev-crossover-products/blinkin/gs/patterns
  */
  public void setColor(double color) {
    blinkin.set(color);
  }
}
