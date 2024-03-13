// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
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
  private NetworkTableEntry m_ty = m_table.getEntry("ty");
  private NetworkTableEntry m_ta = m_table.getEntry("ta");
  private NetworkTableEntry m_tv = m_table.getEntry("tv");

  private int m_nConsecutiveFabrications = 0;
  private MedianFilter m_xFilter = new MedianFilter(LimelightConstants.kFilterSize);
  private MedianFilter m_yFilter = new MedianFilter(LimelightConstants.kFilterSize);
  private double m_filteredX = 0.0;
  private double m_filteredY = 0.0;

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
    double y = m_ty.getDouble(0.0);
    double area = m_ta.getDouble(0.0);
    double v = m_tv.getDouble(0.0);
    boolean notePresent = (v == 1.0);

    if (notePresent) {
      m_nConsecutiveFabrications = 0;
      m_filteredX = m_xFilter.calculate(x);
      m_filteredY = m_yFilter.calculate(y);

      SmartDashboard.putNumber("LimeLightArea", area);
      var distance = getDistance();
      SmartDashboard.putNumber("Distance to Note (in)", distance);
    } else {
      if (m_nConsecutiveFabrications < LimelightConstants.kFilterSize) {
        m_nConsecutiveFabrications++;
        m_filteredX = m_xFilter.calculate(m_filteredX);
        m_filteredY = m_yFilter.calculate(m_filteredY);
      } else {
        m_nConsecutiveFabrications = 0;
        m_xFilter.reset();
        m_yFilter.reset();
        m_filteredX = 0.0;
        m_filteredY = 0.0;
      }
    }
    SmartDashboard.putNumber("LimeLightX", m_filteredX);
    SmartDashboard.putNumber("LimeLightY", m_filteredY);
    SmartDashboard.putBoolean("Note", notePresent);
  }

  public double getX() {
    return m_filteredX;
  }

  public double getY() {
    return m_filteredY;
  }

  private double getDistance() {
    double noteOffsetAngle_Vertical = getY();
    double angleToNoteDegrees = LimelightConstants.kLimelightMountDegrees + noteOffsetAngle_Vertical;
    double angleToNoteRadians = Units.degreesToRadians(angleToNoteDegrees);
    double distanceFromLightToNote = (LimelightConstants.kNoteHeightInches - LimelightConstants.kLimelightLensHeightInches) / Math.tan(angleToNoteRadians);

    return distanceFromLightToNote;
  }
}