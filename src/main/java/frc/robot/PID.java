package frc.robot;

public record PID(double p, double i, double d) {
  public PID {
    assert(p >= 0.0);
    assert(i >= 0.0);
    assert(d >= 0.0);
  }
}