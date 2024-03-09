package frc.robot;

import java.io.IOException;
import java.io.UncheckedIOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class Utilities {
  public static void burnMotor(CANSparkMax motor) {
    if (Constants.kBurnMotors) {
      if (motor.burnFlash() != REVLibError.kOk) {
        throw new UncheckedIOException("Failed to burn motor configuration", new IOException());
      }
    }
  }
}
