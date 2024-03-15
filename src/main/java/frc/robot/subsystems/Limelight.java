// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry m_tx = m_table.getEntry("tx");
  private NetworkTableEntry m_tv = m_table.getEntry("tv");

  private int m_nConsecutiveFabrications = 0;
  private MedianFilter m_xFilter = new MedianFilter(LimelightConstants.kFilterSize);
  private double m_filteredX = 0.0;

  private Relay m_relay = new Relay(LimelightConstants.kRelayPort);

  public Limelight() {
    lightsOff();
  }

  public void lightsOn() {
    m_relay.set(Relay.Value.kOn);
  }

  public void lightsOff() {
    m_relay.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    double x = m_tx.getDouble(0.0);
    double v = m_tv.getDouble(0.0);
    boolean notePresent = (v == 1.0);

    if (notePresent) {
      m_nConsecutiveFabrications = 0;
      m_filteredX = m_xFilter.calculate(x);
    } else {
      if (m_nConsecutiveFabrications < LimelightConstants.kFilterSize) {
        m_nConsecutiveFabrications++;
        m_filteredX = m_xFilter.calculate(m_filteredX);
      } else {
        m_nConsecutiveFabrications = 0;
        m_xFilter.reset();
        m_filteredX = 0.0;
      }
    }
    SmartDashboard.putNumber("Limelight X", m_filteredX);
    SmartDashboard.putBoolean("Note Present", notePresent);
  }

  public double getX() {
    return m_filteredX;
  }
}