/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.mw_lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.utility.ModuleType;

/** All constants for a swerve module. */
public class SwerveModuleConstants {
  /** Supported feedback sensors for the steer motors. */
  public enum SteerFeedbackType {
    RemoteCANcoder,
    /** Requires Pro */
    FusedCANcoder,
    /** Requires Pro */
    SyncCANcoder,
    AnalogEncoder,
    None,
  }

  /** CAN ID of the drive motor. */
  public int DriveMotorId = 0;

  /** CAN ID of the steer motor. */
  public int SteerMotorId = 0;

  /** CAN ID of the CANcoder used for azimuth. */
  public int encoderId = 0;

  /** Offset of the CANcoder in rotations. */
  public double CANcoderOffset = 0;

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
   * The location of this module's wheels relative to the physical center of the robot in meters
   * along the X axis of the robot.
   */
  public double LocationX = 0;

  /**
   * The location of this module's wheels relative to the physical center of the robot in meters
   * along the Y axis of the robot.
   */
  public double LocationY = 0;

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

  /** True if the driving motor is reversed. */
  public boolean DriveMotorInverted = false;

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

  public ModuleType moduleType;

  /**
   * Sets the Module Type
   *
   * @param type type of SDS module being used obtained from Hashtable
   * @return this object
   */
  public SwerveModuleConstants withModuleType(ModuleType type) {
    this.moduleType = type;
    return this;
  }

  /**
   * Sets the CAN ID of the drive motor.
   *
   * @param id CAN ID of the drive motor
   * @return this object
   */
  public SwerveModuleConstants withDriveMotorId(int id) {
    this.DriveMotorId = id;
    return this;
  }

  /**
   * Sets the CAN ID of the steer motor.
   *
   * @param id CAN ID of the steer motor
   * @return this object
   */
  public SwerveModuleConstants withSteerMotorId(int id) {
    this.SteerMotorId = id;
    return this;
  }

  /**
   * Sets the CAN ID of the CANcoder used for azimuth.
   *
   * @param id CAN ID of the CANcoder used for azimuth
   * @return this object
   */
  public SwerveModuleConstants withCANcoderId(int id) {
    this.encoderId = id;
    return this;
  }

  /**
   * Sets the offset of the CANcoder in rotations.
   *
   * @param offset Offset of the CANcoder in rotations
   * @return this object
   */
  public SwerveModuleConstants withCANcoderOffset(double offset) {
    this.CANcoderOffset = offset;
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
  public SwerveModuleConstants withCouplingGearRatio(double ratio) {
    this.CouplingGearRatio = ratio;
    return this;
  }

  /**
   * Sets the radius of the driving wheel in inches.
   *
   * @param radius Radius of the driving wheel in inches
   * @return this object
   */
  public SwerveModuleConstants withWheelRadius(double radius) {
    this.WheelRadius = radius;
    return this;
  }

  /**
   * Sets the location of this module's wheels relative to the physical center of the robot in
   * meters along the X axis of the robot.
   *
   * @param locationXMeters Location of this module's wheels
   * @return this object
   */
  public SwerveModuleConstants withLocationX(double locationXMeters) {
    this.LocationX = locationXMeters;
    return this;
  }

  /**
   * Sets the location of this module's wheels relative to the physical center of the robot in
   * meters along the Y axis of the robot.
   *
   * @param locationYMeters Location of this module's wheels
   * @return this object
   */
  public SwerveModuleConstants withLocationY(double locationYMeters) {
    this.LocationY = locationYMeters;
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
  public SwerveModuleConstants withSteerMotorGains(Slot0Configs gains) {
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
  public SwerveModuleConstants withDriveMotorGains(Slot0Configs gains) {
    this.DriveMotorGains = gains;
    return this;
  }

  /**
   * Sets closed-loop output type to use for the steer motors.
   *
   * @param outputType Closed-loop output type to use for the steer motors
   * @return this object
   */
  public SwerveModuleConstants withSteerMotorClosedLoopOutput(ClosedLoopOutputType outputType) {
    this.SteerMotorClosedLoopOutput = outputType;
    return this;
  }

  /**
   * Sets closed-loop output type to use for the drive motors.
   *
   * @param outputType Closed-loop output type to use for the drive motors
   * @return this object
   */
  public SwerveModuleConstants withDriveMotorClosedLoopOutput(ClosedLoopOutputType outputType) {
    this.DriveMotorClosedLoopOutput = outputType;
    return this;
  }

  /**
   * Sets the maximum amount of stator current the drive motors can apply without slippage.
   *
   * @param slipCurrent Maximum amount of stator current
   * @return this object
   */
  public SwerveModuleConstants withSlipCurrent(double slipCurrent) {
    this.SlipCurrent = slipCurrent;
    return this;
  }

  /**
   * Sets whether the steering motor is reversed from the CANcoder.
   *
   * @param steerMotorInverted True if the steering motor is reversed from the CANcoder
   * @return this object
   */
  public SwerveModuleConstants withSteerMotorInverted(boolean steerMotorInverted) {
    this.SteerMotorInverted = steerMotorInverted;
    return this;
  }

  /**
   * Sets whether the driving motor is reversed.
   *
   * @param driveMotorInverted True if the driving motor is reversed
   * @return this object
   */
  public SwerveModuleConstants withDriveMotorInverted(boolean driveMotorInverted) {
    this.DriveMotorInverted = driveMotorInverted;
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
  public SwerveModuleConstants withSpeedAt12VoltsMps(double speedAt12VoltsMps) {
    this.SpeedAt12VoltsMps = speedAt12VoltsMps;
    return this;
  }

  /**
   * Sets the simulated azimuthal inertia in kilogram meters squared.
   *
   * @param steerInertia Azimuthal inertia in kilogram meters squared
   * @return this object
   */
  public SwerveModuleConstants withSteerInertia(double steerInertia) {
    this.SteerInertia = steerInertia;
    return this;
  }

  /**
   * Sets the simulated drive inertia in kilogram meters squared.
   *
   * @param driveInertia Drive inertia in kilogram meters squared
   * @return this object
   */
  public SwerveModuleConstants withDriveInertia(double driveInertia) {
    this.DriveInertia = driveInertia;
    return this;
  }

  /**
   * Sets the simulated steer voltage required to overcome friction.
   *
   * @param voltage Steer voltage required to overcome friction
   * @return this object
   */
  public SwerveModuleConstants withSteerFrictionVoltage(double voltage) {
    this.SteerFrictionVoltage = voltage;
    return this;
  }

  /**
   * Sets the simulated drive voltage required to overcome friction.
   *
   * @param voltage Drive voltage required to overcome friction
   * @return this object
   */
  public SwerveModuleConstants withDriveFrictionVoltage(double voltage) {
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
  public SwerveModuleConstants withFeedbackSource(SteerFeedbackType source) {
    this.FeedbackSource = source;
    return this;
  }
}
