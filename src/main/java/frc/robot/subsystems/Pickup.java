// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants.PickupConstants;
import frc.robot.commands.SetDefaultPickup;
import monologue.Annotations.Log;
import monologue.Logged;

public class Pickup extends Subsystem {

  private TalonFX roller_motor_;
  private TalonFX pivot_motor_;

  private PositionVoltage pivot_request_;

  private StructPublisher<Pose3d> pickup_pub_;

  public enum PickupMode {
    INTAKE,
    FLUSH_OUT,
    STATION,
    DEPLOYED,
    CLIMB
  }

  // Singleton pattern
  private static Pickup pickup_instance_ = null;

  public static Pickup getInstance() {
    if (pickup_instance_ == null) {
      pickup_instance_ = new Pickup();
    }
    return pickup_instance_;
  }

  /** Class Members */
  private PickupPeriodicIo io_;

  private Pickup() {
    // Create io object first in subsystem configuration
    io_ = new PickupPeriodicIo();

    roller_motor_ = new TalonFX(PickupConstants.INTAKE_ID);
    pivot_motor_ = new TalonFX(PickupConstants.PIVOT_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0 = PickupConstants.PICKUP_GAINS;
    config.CurrentLimits.StatorCurrentLimit = PickupConstants.StatorCurrentLimit;
    config.Feedback.SensorToMechanismRatio = PickupConstants.SENSOR_TO_MECHANISM_RATIO;
    pivot_motor_.getConfigurator().apply(config);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    roller_motor_.getConfigurator().apply(config);

    pivot_request_ = new PositionVoltage(0);

    SmartDashboard.putData(
        "Subsystems/Pickup/Zero Ground",
        Commands.runOnce(() -> setPivotPosition(PickupConstants.PIVOT_DEPLOYED_ANGLE)));
    SmartDashboard.putData(
        "Subsystems/Pickup/Zero Station",
        Commands.runOnce(() -> setPivotPosition(PickupConstants.PIVOT_STATION_ANGLE)));

    pickup_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Components/Pickup", Pose3d.struct)
            .publish();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {
    setDefaultCommand(new SetDefaultPickup());
  }

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_pivot_angle_ =
        Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble())
            .minus(PickupConstants.PIVOT_OFFSET);
    io_.current_roller_outout_ = roller_motor_.getDutyCycle().getValueAsDouble();
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    switch (io_.current_mode_) {
      case INTAKE:
        io_.target_roller_output_ = PickupConstants.INTAKE_IN_SPEED;
        io_.target_pivot_angle_ = PickupConstants.PIVOT_DEPLOYED_ANGLE;
        break;
      case DEPLOYED:
        io_.target_roller_output_ = 0;
        io_.target_pivot_angle_ = PickupConstants.PIVOT_DEPLOYED_ANGLE;
        break;
      case FLUSH_OUT:
        io_.target_roller_output_ = PickupConstants.INTAKE_OUT_SPEED;
        io_.target_pivot_angle_ = PickupConstants.PIVOT_DEPLOYED_ANGLE;
        break;
      case STATION:
        io_.target_roller_output_ = 0;
        io_.target_pivot_angle_ = PickupConstants.PIVOT_STATION_ANGLE;
        break;
      case CLIMB:
        io_.target_roller_output_ = 0;
        io_.target_pivot_angle_ = PickupConstants.PIVOT_CLIMB_ANGLE;
      default:
        io_.target_roller_output_ = 0;
        break;
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    roller_motor_.set(io_.target_roller_output_);
    pivot_motor_.setControl(
        pivot_request_.withPosition(
            io_.target_pivot_angle_.plus(PickupConstants.PIVOT_OFFSET).getRotations()));
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putString("Subsystems/Pickup/Mode", io_.current_mode_.toString());
    SmartDashboard.putNumber(
        "Subsystems/Pickup/Current Angle", io_.current_pivot_angle_.getDegrees());
    SmartDashboard.putNumber(
        "Subsystems/Pickup/Target Angle", io_.target_pivot_angle_.getDegrees());
    updateMechanism();
  }

  private void updateMechanism() {
    pickup_pub_.set(
        new Pose3d(
            -Units.inchesToMeters(11),
            0,
            Units.inchesToMeters(8.25),
            new Rotation3d(0, io_.current_pivot_angle_.getRadians(), 0)));
  }

  /**
   * Sets the current mode of the Pickup
   *
   * @param target_mode new mode that is being set
   */
  public void setPickupMode(PickupMode target_mode) {
    io_.current_mode_ = target_mode;
  }

  public boolean isAtTarget() {
    return Util.epislonEquals(
        io_.current_pivot_angle_, io_.target_pivot_angle_, PickupConstants.PIVOT_THRESHOLD);
  }

  /**
   * @return the current mode of the Pickup
   */
  public PickupMode getPickupMode() {
    return io_.current_mode_;
  }

  /** Resets to the zero position of the pivot motor */
  public void setPivotPosition(Rotation2d value) {
    pivot_motor_.setPosition(value.getRotations());
  }

  public class PickupPeriodicIo implements Logged {
    @Log.File public PickupMode current_mode_ = PickupMode.DEPLOYED;
    @Log.File public double target_roller_output_ = 0;
    @Log.File public double current_roller_outout_ = 0;
    @Log.File public Rotation2d current_pivot_angle_ = new Rotation2d();
    @Log.File public Rotation2d target_pivot_angle_ = new Rotation2d();
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
