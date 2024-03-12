package frc.robot;

import com.revrobotics.SparkPIDController;

public record PIDF(double p, double i, double d, double ff) {
  public PIDF {
    assert(p >= 0.0);
    assert(i >= 0.0);
    assert(d >= 0.0);
    assert(ff >= 0.0);
  }

  public PIDF(double p, double i, double d) {
    this(p, i, d, 0.0);
  }

  public void controllerSet(SparkPIDController pidController, int slotID) {
    pidController.setP(p, slotID);
    pidController.setI(i, slotID);
    pidController.setD(d, slotID);
    pidController.setFF(ff, slotID);
  }

  public void controllerSet(SparkPIDController pidController) {
    controllerSet(pidController, 0);
  }
}