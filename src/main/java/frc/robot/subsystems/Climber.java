// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends Subsystem {

  private TalonFX strap_motor_;
  private Spark prong_motor_;
  private Spark arm_motor_;

  public enum ClimberMode {
    ENABLED,
    DISABLED
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

    strap_motor_ = new TalonFX(Constants.ClimberConstants.STRAP_ID);
    prong_motor_ = new Spark(Constants.ClimberConstants.PRONG_ID);
    arm_motor_ = new Spark(Constants.ClimberConstants.ARM_ID);

    strap_motor_.setNeutralMode(NeutralModeValue.Brake);
    strap_motor_.setInverted(Constants.ClimberConstants.STRAP_INVERSION);

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
  public void readPeriodicInputs(double timestamp) {}

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {}

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    if (io_.current_mode_ == ClimberMode.ENABLED) {
      strap_motor_.set(io_.strap_motor_target);
      prong_motor_.set(io_.prong_motor_target);
      arm_motor_.set(io_.arm_motor_target);
    } else {
      strap_motor_.set(0);
      prong_motor_.set(0);
      arm_motor_.set(0);
    }
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Subsystems/Climber/strap_motor_target", io_.strap_motor_target);
    SmartDashboard.putNumber("Subsystems/Climber/prong_motor_target", io_.prong_motor_target);
    SmartDashboard.putNumber("Subsystems/Climber/arm_motor_target", io_.arm_motor_target);
    SmartDashboard.putString("Subsystems/Climber/current_mode_", io_.current_mode_.toString());
  }

  /**
   * Sets the current mode of climber
   *
   * @param target_mode new mode that is being set
   */
  public void setClimberMode(ClimberMode target_mode) {
    io_.current_mode_ = target_mode;
  }

  public void setStrapSpeed(double target) {
    io_.strap_motor_target = target;
  }

  public void setProngSpeed(double target) {
    io_.prong_motor_target = target;
  }

  public void setArmSpeed(double target) {
    io_.arm_motor_target = target;
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File public double strap_motor_target = 0;
    @Log.File public double prong_motor_target = 0;
    @Log.File public double arm_motor_target = 0;
    @Log.File public ClimberMode current_mode_ = ClimberMode.DISABLED;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
