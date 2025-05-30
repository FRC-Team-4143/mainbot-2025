/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.mw_lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.mw_lib.swerve.utility.ModuleType;

/**
 * Constants that are common across the swerve modules, used for creating instances of
 * module-specific {@link SwerveModuleConstants}.
 */
public class SwerveModuleConstantsFactory {
  /** Gear ratio between the drive motor and the wheel. */
  public double DriveMotorGearRatio = 0;

  /**
   * Gear ratio between the steer motor and the CANcoder. For example, the SDS Mk4 has a steering
   * ratio of 12.8.
   */
  public double SteerMotorGearRatio = 0;

  /**
   * Coupled gear ratio between the CANcoder and the drive motor.
   *
   * <p>For a typical swerve module, the azimuth turn motor also drives the wheel a nontrivial
   * amount, which affects the accuracy of odometry and control. This ratio represents the number of
   * rotations of the drive motor caused by a rotation of the azimuth.
   */
  public double CouplingGearRatio = 0;

  /** Radius of the driving wheel in inches. */
  public double WheelRadius = 0;

  /**
   * The steer motor closed-loop gains.
   *
   * <p>The steer motor uses the control ouput type specified by {@link #SteerMotorClosedLoopOutput}
   * and any {@link SwerveModule.SteerRequestType}.
   */
  public Slot0Configs SteerMotorGains = new Slot0Configs();

  /**
   * The drive motor closed-loop gains.
   *
   * <p>When using closed-loop control, the drive motor uses the control output type specified by
   * {@link #DriveMotorClosedLoopOutput} and any closed-loop {@link SwerveModule.DriveRequestType}.
   */
  public Slot0Configs DriveMotorGains = new Slot0Configs();

  /** The closed-loop output type to use for the steer motors. */
  public ClosedLoopOutputType SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;

  /** The closed-loop output type to use for the drive motors. */
  public ClosedLoopOutputType DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;

  /** The maximum amount of stator current the drive motors can apply without slippage. */
  public double SlipCurrent = 10;

  /** True if the steering motor is reversed from the CANcoder. */
  public boolean SteerMotorInverted = false;

  /**
   * When using open-loop drive control, this specifies the speed at which the robot travels when
   * driven with 12 volts, in meters per second. This is used to approximate the output for a
   * desired velocity. If using closed loop control, this value is ignored.
   */
  public double SpeedAt12VoltsMps = 0;

  /** Sim-specific constants * */
  /** Simulated azimuthal inertia in kilogram meters squared. */
  public double SteerInertia = 0.00001;

  /** Simulated drive inertia in kilogram meters squared. */
  public double DriveInertia = 0.001;

  /** Simulated steer voltage required to overcome friction. */
  public double SteerFrictionVoltage = 0.25;

  /** Simulated drive voltage required to overcome friction. */
  public double DriveFrictionVoltage = 0.25;

  /**
   * Choose how the feedback sensors should be configured.
   *
   * <p>If the robot does not support Pro, then this should remain as RemoteCANcoder. Otherwise,
   * users have the option to use either FusedCANcoder or SyncCANcoder depending on if there is a
   * risk that the CANcoder can fail in a way to provide "good" data.
   */
  public SteerFeedbackType FeedbackSource = SteerFeedbackType.RemoteCANcoder;

  /**
   * Sets the gear ratio between the drive motor and the wheel.
   *
   * @param ratio Gear ratio between the drive motor and the wheel
   * @return this object
   */
  public SwerveModuleConstantsFactory withDriveMotorGearRatio(double ratio) {
    this.DriveMotorGearRatio = ratio;
    return this;
  }

  /**
   * Sets the gear ratio between the steer motor and the CANcoder. For example, the SDS Mk4 has a
   * steering ratio of 12.8.
   *
   * @param ratio Gear ratio between the steer motor and the CANcoder
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerMotorGearRatio(double ratio) {
    this.SteerMotorGearRatio = ratio;
    return this;
  }

  /**
   * Sets the coupled gear ratio between the CANcoder and the drive motor.
   *
   * <p>For a typical swerve module, the azimuth turn motor also drives the wheel a nontrivial
   * amount, which affects the accuracy of odometry and control. This ratio represents the number of
   * rotations of the drive motor caused by a rotation of the azimuth.
   *
   * @param ratio Coupled gear ratio between the CANcoder and the drive motor
   * @return this object
   */
  public SwerveModuleConstantsFactory withCouplingGearRatio(double ratio) {
    this.CouplingGearRatio = ratio;
    return this;
  }

  /**
   * Sets the radius of the driving wheel in inches.
   *
   * @param radius Radius of the driving wheel in inches
   * @return this object
   */
  public SwerveModuleConstantsFactory withWheelRadius(double radius) {
    this.WheelRadius = radius;
    return this;
  }

  /**
   * Sets the steer motor closed-loop gains.
   *
   * <p>The steer motor uses the control ouput type specified by {@link #SteerMotorClosedLoopOutput}
   * and any {@link SwerveModule.SteerRequestType}.
   *
   * @param gains Steer motor closed-loop gains
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerMotorGains(Slot0Configs gains) {
    this.SteerMotorGains = gains;
    return this;
  }

  /**
   * Sets the drive motor closed-loop gains.
   *
   * <p>When using closed-loop control, the drive motor uses the control output type specified by
   * {@link #DriveMotorClosedLoopOutput} and any closed-loop {@link SwerveModule.DriveRequestType}.
   *
   * @param gains Drive motor closed-loop gains
   * @return this object
   */
  public SwerveModuleConstantsFactory withDriveMotorGains(Slot0Configs gains) {
    this.DriveMotorGains = gains;
    return this;
  }

  /**
   * Sets closed-loop output type to use for the steer motors.
   *
   * @param outputType Closed-loop output type to use for the steer motors
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerMotorClosedLoopOutput(
      ClosedLoopOutputType outputType) {
    this.SteerMotorClosedLoopOutput = outputType;
    return this;
  }

  /**
   * Sets closed-loop output type to use for the drive motors.
   *
   * @param outputType Closed-loop output type to use for the drive motors
   * @return this object
   */
  public SwerveModuleConstantsFactory withDriveMotorClosedLoopOutput(
      ClosedLoopOutputType outputType) {
    this.DriveMotorClosedLoopOutput = outputType;
    return this;
  }

  /**
   * Sets the maximum amount of stator current the drive motors can apply without slippage.
   *
   * @param slipCurrent Maximum amount of stator current
   * @return this object
   */
  public SwerveModuleConstantsFactory withSlipCurrent(double slipCurrent) {
    this.SlipCurrent = slipCurrent;
    return this;
  }

  /**
   * Sets whether the steering motor is reversed from the CANcoder.
   *
   * @param steerMotorInverted True if the steering motor is reversed from the CANcoder
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerMotorInverted(boolean steerMotorInverted) {
    this.SteerMotorInverted = steerMotorInverted;
    return this;
  }

  /**
   * When using open-loop drive control, this specifies the speed at which the robot travels when
   * driven with 12 volts, in meters per second. This is used to approximate the output for a
   * desired velocity. If using closed loop control, this value is ignored.
   *
   * @param speedAt12VoltsMps Speed at which the robot travels when driven with 12 volts, in meters
   *     per second
   * @return this object
   */
  public SwerveModuleConstantsFactory withSpeedAt12VoltsMps(double speedAt12VoltsMps) {
    this.SpeedAt12VoltsMps = speedAt12VoltsMps;
    return this;
  }

  /**
   * Sets the simulated azimuthal inertia in kilogram meters squared.
   *
   * @param steerInertia Azimuthal inertia in kilogram meters squared
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerInertia(double steerInertia) {
    this.SteerInertia = steerInertia;
    return this;
  }

  /**
   * Sets the simulated drive inertia in kilogram meters squared.
   *
   * @param driveInertia Drive inertia in kilogram meters squared
   * @return this object
   */
  public SwerveModuleConstantsFactory withDriveInertia(double driveInertia) {
    this.DriveInertia = driveInertia;
    return this;
  }

  /**
   * Sets the simulated steer voltage required to overcome friction.
   *
   * @param voltage Steer voltage required to overcome friction
   * @return this object
   */
  public SwerveModuleConstantsFactory withSteerFrictionVoltage(double voltage) {
    this.SteerFrictionVoltage = voltage;
    return this;
  }

  /**
   * Sets the simulated drive voltage required to overcome friction.
   *
   * @param voltage Drive voltage required to overcome friction
   * @return this object
   */
  public SwerveModuleConstantsFactory withDriveFrictionVoltage(double voltage) {
    this.DriveFrictionVoltage = voltage;
    return this;
  }

  /**
   * Chooses how the feedback sensors should be configured.
   *
   * <p>If the robot does not support Pro, then this should remain as RemoteCANcoder. Otherwise,
   * users have the option to use either FusedCANcoder or SyncCANcoder depending on if there is a
   * risk that the CANcoder can fail in a way to provide "good" data.
   *
   * @param source The feedback sensor source
   * @return this object
   */
  public SwerveModuleConstantsFactory withFeedbackSource(SteerFeedbackType source) {
    this.FeedbackSource = source;
    return this;
  }

  /**
   * Creates the constants for a swerve module with the given properties.
   *
   * @param steerId CAN ID of the steer motor
   * @param driveId CAN ID of the drive motor
   * @param cancoderId CAN ID of the CANcoder used for azimuth
   * @param cancoderOffset Offset of the CANcoder in rotations
   * @param locationX The location of this module's wheels relative to the physical center of the
   *     robot in meters along the X axis of the robot
   * @param locationY The location of this module's wheels relative to the physical center of the
   *     robot in meters along the Y axis of the robot
   * @param driveMotorReversed True if the driving motor is reversed
   * @return Constants for the swerve module
   */
  public SwerveModuleConstants createModuleConstants(
      int steerId,
      int driveId,
      int cancoderId,
      double cancoderOffset,
      double locationX,
      double locationY,
      boolean driveMotorReversed,
      ModuleType moduleType) {
    return new SwerveModuleConstants()
        .withModuleType(moduleType)
        .withSteerMotorId(steerId)
        .withDriveMotorId(driveId)
        .withCANcoderId(cancoderId)
        .withCANcoderOffset(cancoderOffset)
        .withLocationX(locationX)
        .withLocationY(locationY)
        .withCouplingGearRatio(CouplingGearRatio)
        .withWheelRadius(WheelRadius)
        .withSlipCurrent(SlipCurrent)
        .withSteerMotorGains(SteerMotorGains)
        .withDriveMotorGains(DriveMotorGains)
        .withSteerMotorClosedLoopOutput(SteerMotorClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(DriveMotorClosedLoopOutput)
        .withSteerMotorInverted(SteerMotorInverted)
        .withDriveMotorInverted(driveMotorReversed)
        .withSpeedAt12VoltsMps(SpeedAt12VoltsMps)
        .withSteerInertia(SteerInertia)
        .withDriveInertia(DriveInertia)
        .withSteerFrictionVoltage(SteerFrictionVoltage)
        .withDriveFrictionVoltage(DriveFrictionVoltage)
        .withFeedbackSource(FeedbackSource);
  }
}
