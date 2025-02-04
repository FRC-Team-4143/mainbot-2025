// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.controls.TalonFXTuner;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.OI;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends Subsystem {

  private TalonFX climber_motor_;
  private TalonFXConfiguration climber_config_;
  private PositionVoltage climber_request_;
  private TalonFXTuner climber_tuner_;

  enum ClimberMode {
    DEPLOYED,
    RETRACTED
  }

  // Singleton pattern
  private static Climber climber_instance_ = null;

  public static Climber getInstance() {
    if (climber_instance_ == null) {
      climber_instance_ = new Climber();
    }
    return climber_instance_;
  }

  /** Class Members */
  private ClimberPeriodicIo io_;

  private Climber() {
    // Create io object first in subsystem configuration
    io_ = new ClimberPeriodicIo();
    climber_motor_ = new TalonFX(ClimberConstants.CLIMBER_ID);
    climber_request_ = new PositionVoltage(0).withSlot(0);

    climber_config_ = new TalonFXConfiguration();
    climber_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climber_config_.Slot0 = ClimberConstants.CLIMBER_GAINS;

    climber_motor_.getConfigurator().apply(climber_config_);

    climber_tuner_ = new TalonFXTuner(climber_motor_, "Climber", this);
    bindTuner(climber_tuner_, 5, 10);

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {}

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_amps_ = climber_motor_.getSupplyCurrent().getValue().in(Amps);
    io_.current_rotations_ = climber_motor_.getPosition().getValueAsDouble();
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    switch (io_.current_mode_) {
      case DEPLOYED:
        io_.target_rotations_ = ClimberConstants.DEPLOYED_ROTATIONS;
        break;
      case RETRACTED:
        io_.target_rotations_ = ClimberConstants.RETRACTED_ROTATIONS;
        break;
      default:
        io_.target_rotations_ = ClimberConstants.RETRACTED_ROTATIONS;
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
    climber_motor_.setControl(climber_request_.withPosition(io_.target_rotations_));
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putString("Debug/Climber/Mode", io_.current_mode_.toString());
    SmartDashboard.putNumber("Debug/Climber/Target Rotations", io_.target_rotations_);
    SmartDashboard.putNumber("Debug/Climber/Current Rotations", io_.current_rotations_);
  }

  /**
   * Sets the current mode of climber
   *
   * @param target_mode new mode that is being set
   */
  public void setClimberMode(ClimberMode target_mode) {
    io_.current_mode_ = target_mode;
  }

  /**
   * @return Returns current rotations in rotations
   */
  public double getCurrentRotations() {
    return io_.current_rotations_;
  }

  /**
   * @return Returns current amps in amps
   */
  public double getCurrentAmps() {
    return io_.current_amps_;
  }

  /** Resets to the zero position of the climber motor */
  public void resetClimberPosition() {
    climber_motor_.setPosition(0);
  }

  public void bindTuner(TalonFXTuner tuner, double pos1, double pos2) {
    tuner.bindSetpoint(new MotionMagicVoltage(pos1), OI.getDriverJoystickAButtonTrigger());
    tuner.bindSetpoint(new MotionMagicVoltage(pos2), OI.getDriverJoystickYButtonTrigger());
    tuner.bindSetpoint(new VoltageOut(-1), OI.getDriverJoystickXButtonTrigger());
    tuner.bindSetpoint(new VoltageOut(1), OI.getDriverJoystickBButtonTrigger());
    tuner.bindDynamicForward(OI.getOperatorJoystickAButtonTrigger());
    tuner.bindDynamicReverse(OI.getOperatorJoystickBButtonTrigger());
    tuner.bindQuasistaticForward(OI.getOperatorJoystickXButtonTrigger());
    tuner.bindQuasistaticReverse(OI.getOperatorJoystickYButtonTrigger());
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File public double current_rotations_ = 0;
    @Log.File public ClimberMode current_mode_ = ClimberMode.RETRACTED;
    @Log.File public double current_amps_;
    @Log.File public double target_rotations_ = 0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
