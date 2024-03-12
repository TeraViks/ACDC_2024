// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.TunablePIDF;
import frc.robot.Utilities;
import frc.robot.ValueCache;

public class SwerveModule {
  public static TunablePIDF tunableTeleopTurningPIDF =
    new TunablePIDF("TeleopTurning", SwerveModuleConstants.kTeleopTurningPIDF);

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private int m_PIDFSlotID = SwerveModuleConstants.kDefaultPIDFSlotID;

  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_absoluteRotationEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final Rotation2d m_absoluteRotationEncoderOffset;
  private Rotation2d m_turningEncoderOffset;

  private final ValueCache<Double> m_drivePositionCache;
  private final ValueCache<Double> m_driveVelocityCache;
  private final ValueCache<Double> m_absoluteRotationCache;
  private final ValueCache<Double> m_turningCache;
  private Rotation2d m_prevAngle;
  private double m_lastViableDrivePosition = 0.0;
  private double m_lastViableDriveVelocity = 0.0;
  private double m_lastViableTurningPosition = 0.0;

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      boolean turningEncoderReversed,
      Rotation2d encoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setClosedLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate);
    m_driveMotor.setInverted(driveMotorReversed);
    m_driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit);
    m_driveMotor.setIdleMode(IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivePositionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveVelocityConversionFactor);
    m_drivePositionCache = new ValueCache<Double>(this::getPlausibleDrivePosition, SwerveModuleConstants.kValueCacheTtlMicroseconds);
    m_driveVelocityCache = new ValueCache<Double>(this::getPlausibleDriveVelocity, SwerveModuleConstants.kValueCacheTtlMicroseconds);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    SwerveModuleConstants.kAutoDrivePIDF.controllerSet(
      m_drivePIDController, SwerveModuleConstants.kAutoPIDFSlotID);
    SwerveModuleConstants.kTeleopDrivePIDF.controllerSet(
      m_drivePIDController, SwerveModuleConstants.kTeleopPIDFSlotID);

    m_absoluteRotationEncoderOffset = encoderOffset;

    m_absoluteRotationEncoder = new CANcoder(turningEncoderChannel);
    var turningEncoderConfigurator = m_absoluteRotationEncoder.getConfigurator();
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = turningEncoderReversed
      ? SensorDirectionValue.Clockwise_Positive
      : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    turningEncoderConfigurator.apply(encoderConfig);
    m_absoluteRotationCache =
      new ValueCache<Double>(() -> {
        return m_absoluteRotationEncoder.getAbsolutePosition().getValue();
      }, SwerveModuleConstants.kValueCacheTtlMicroseconds);
    Utilities.burnMotor(m_driveMotor);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setClosedLoopRampRate(SwerveModuleConstants.kTurningMotorRampRate);
    m_turningMotor.setInverted(turningMotorReversed);
    m_turningMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningPositionConversionFactor);
    m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningVelocityConversionFactor);
    // Stabilize encoder readings before initializing the cache.
    m_turningEncoder.setPosition(0.0);
    while (
      Math.abs(m_turningEncoder.getPosition())
      > SwerveModuleConstants.kTurningEncoderStabilizeToleranceRadians
    ) {Thread.yield();}
    m_turningCache = new ValueCache<Double>(this::getPlausibleTurningPosition,
      SwerveModuleConstants.kValueCacheTtlMicroseconds);
    updateTurningEncoderOffset();
    m_prevAngle = getUnconstrainedRotation2d();

    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    SwerveModuleConstants.kAutoTurningPIDF.controllerSet(
      m_turningPIDController, SwerveModuleConstants.kAutoPIDFSlotID);
    SwerveModuleConstants.kTeleopTurningPIDF.controllerSet(
      m_turningPIDController, SwerveModuleConstants.kTeleopPIDFSlotID);
    Utilities.burnMotor(m_turningMotor);
  }

  private double getPlausibleDrivePosition() {
    double position = m_driveEncoder.getPosition();
    if (m_driveMotor.getLastError() == REVLibError.kOk) {
      m_lastViableDrivePosition = position;
    }
    return m_lastViableDrivePosition;
  }

  private double getPlausibleDriveVelocity() {
    double velocity = m_driveEncoder.getVelocity();
    if (m_driveMotor.getLastError() == REVLibError.kOk) {
      m_lastViableDriveVelocity = velocity;
    }
    return m_lastViableDriveVelocity;
  }

  private double getPlausibleTurningPosition() {
    double position = m_turningEncoder.getPosition();
    if (m_turningMotor.getLastError() == REVLibError.kOk) {
      m_lastViableTurningPosition = position;
    }
    return m_lastViableTurningPosition;
  }

  public void setPIDSlotID(int slotID) {
    m_PIDFSlotID = slotID;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveVelocityCache.get(),
      getRotation2d()
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_drivePositionCache.get(),
      getRotation2d()
    );
  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees. Note that providing
    // m_prevAngle as the current angle is not as truthful as providing getRotation2d(), because the
    // module may not have yet reached the angle it was most recently commanded to. However, there
    // is inherent instability in the algorithm if we tell the truth using getRotation2d(), because
    // commanding a position and reading current position are both asynchronous.
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_prevAngle);

    m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, m_PIDFSlotID);

    if (!state.angle.equals(m_prevAngle)) {
      // deltaAngle is in [-pi..pi], which is added (intentionally unconstrained) to m_prevAngle.
      // This causes the module to turn e.g. 4 degrees rather than -356 degrees.
      Rotation2d deltaAngle = state.angle.minus(m_prevAngle);
      // Avoid Rotation2d.plus() here, since it constrains the result to [-pi..pi].
      Rotation2d angle = Rotation2d.fromRadians(m_prevAngle.getRadians() + deltaAngle.getRadians());
      // Take care to cancel out the encoder offset when setting the position.
      m_turningPIDController.setReference(angle.getRadians() + m_turningEncoderOffset.getRadians(),
        ControlType.kPosition, m_PIDFSlotID);
      m_prevAngle = angle;
    }
  }

  private Rotation2d getAbsoluteRotation2d() {
    double absolutePositionRotations = m_absoluteRotationCache.get();
    double absolutePositionRadians = Units.rotationsToRadians(absolutePositionRotations);
    return new Rotation2d(absolutePositionRadians).minus(m_absoluteRotationEncoderOffset);
  }

  /**
   * Update relative turning encoder offset to correspond to the absolute turning encoder. This
   * should only be done when the robot is (at least nearly) stationary. Unfortunately, the
   * relative encoder in the NEO isn't very accurate, and angle mismatches up to ~15 degrees
   * commonly occur in the absence of such updates.
   */
  public void updateTurningEncoderOffset() {
    Rotation2d absolute = getAbsoluteRotation2d();
    double relativeRadians = m_turningCache.get();
    m_turningEncoderOffset = Rotation2d.fromRadians(relativeRadians).minus(absolute);
  }

  private Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(m_turningCache.get()).minus(m_turningEncoderOffset);
  }

  private Rotation2d getUnconstrainedRotation2d() {
    // Avoid Rotation2d.minus() here, since it constrains the result to [-pi..pi].
    return Rotation2d.fromRadians(m_turningCache.get() - m_turningEncoderOffset.getRadians());
  }
}
