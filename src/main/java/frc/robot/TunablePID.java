package frc.robot;

public class TunablePID {
  private final TunableDouble m_p, m_i, m_d;
  private PID m_pid;

  public TunablePID(String key, PID defaultValue) {
    m_p = new TunableDouble(key + ".P", defaultValue.p());
    m_i = new TunableDouble(key + ".I", defaultValue.i());
    m_d = new TunableDouble(key + ".D", defaultValue.d());
  }

  public PID get() {
    update();
    return m_pid;
  }

  private void update() {
    double p = m_p.get();
    double i = m_i.get();
    double d = m_d.get();
    m_pid = new PID(p, i, d);
  }

  public boolean hasChanged() {
    return m_p.hasChanged() || m_i.hasChanged() || m_d.hasChanged();
  }
}