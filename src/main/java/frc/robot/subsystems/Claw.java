// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Claw extends Subsystem {

  private TalonFX wheel_motor_;
  private TalonFXConfiguration wheel_config_;

  public enum ClawMode {
    SHOOT,
    LOAD,
    IDLE,
    BLAST
  }

  public enum GamePiece {
    CORAL,
    ALGAE
  }

  // Singleton pattern
  private static Claw claw_instance_ = null;

  private Debouncer coral_debouncer_ = new Debouncer(0.30, Debouncer.DebounceType.kBoth);
  private Debouncer algae_debouncer_ = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public static Claw getInstance() {
    if (claw_instance_ == null) {
      claw_instance_ = new Claw();
    }
    return claw_instance_;
  }

  /** Class Members */
  private ClawPeriodicIo io_;

  private Claw() {
    // Create io object first in subsystem configuration
    io_ = new ClawPeriodicIo();

    wheel_motor_ = new TalonFX(ClawConstants.WHEEL_MOTOR_ID, "CANivore");
    wheel_config_ = new TalonFXConfiguration();
    wheel_config_.CurrentLimits.StatorCurrentLimit = ClawConstants.STATOR_CURRENT_LIMIT;
    wheel_config_.CurrentLimits.StatorCurrentLimitEnable = true;
    wheel_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wheel_config_.MotorOutput.Inverted = ClawConstants.WHEEL_MOTOR_INVERTED;

    wheel_motor_.getConfigurator().apply(wheel_config_);

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
    io_.current_output_ = wheel_motor_.getSupplyCurrent().getValue().in(Amps);
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    if (io_.game_piece_ == GamePiece.CORAL) {
      switch (io_.claw_mode_) {
        case BLAST:
          io_.wheel_output_ = ClawConstants.WHEEL_CORAL_BLAST_SPEED;
          break;
        case SHOOT:
          io_.wheel_output_ = ClawConstants.WHEEL_CORAL_SHOOT_SPEED;
          break;
        case LOAD:
          io_.wheel_output_ = ClawConstants.WHEEL_LOAD_SPEED;
          break;
        case IDLE:
        default:
          io_.wheel_output_ = 0;
          break;
      }
    } else {
      switch (io_.claw_mode_) {
        case BLAST:
          io_.wheel_output_ = -ClawConstants.WHEEL_ALGAE_BLAST_SPEED;
          break;
        case SHOOT:
          io_.wheel_output_ = -ClawConstants.WHEEL_ALGAE_SHOOT_SPEED;
          break;
        case LOAD:
          io_.wheel_output_ = -ClawConstants.WHEEL_LOAD_SPEED;
          break;
        case IDLE:
        default:
          io_.wheel_output_ = ClawConstants.ALGAE_IDLE_SPEED;
          break;
      }
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    wheel_motor_.set(io_.wheel_output_);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putString("Subsystems/Claw/Mode", io_.claw_mode_.toString());
    SmartDashboard.putNumber("Subsystems/Claw/Current_Output", io_.current_output_);
    SmartDashboard.putBoolean("Subsystems/Claw/Has Algae", hasAlgae());
    SmartDashboard.putBoolean("Subsystems/Claw/Has Coral (On True)", hasCoral());
    SmartDashboard.putString(
        "Subsystems/Claw/Game Piece Mode",
        (io_.game_piece_ == GamePiece.CORAL)
            ? Constants.ClawConstants.CORAL_COLOR
            : Constants.ClawConstants.ALGAE_COLOR);
  }

  /**
   * Set the mode of the claw
   *
   * @param claw_mode the new mode to be set
   */
  public void setClawMode(ClawMode claw_mode) {
    if (io_.enable_blast_ && claw_mode == ClawMode.SHOOT) {
      io_.claw_mode_ = ClawMode.BLAST;
    } else {
      io_.claw_mode_ = claw_mode;
    }
  }

  public void enableBlastMode() {
    io_.enable_blast_ = true;
  }

  public void disableBlastMode() {
    io_.enable_blast_ = false;
  }

  public void setGamePiece(GamePiece gamePiece) {
    io_.game_piece_ = gamePiece;
  }

  public GamePiece getGamePieceMode() {
    return io_.game_piece_;
  }

  public Command toggleGamePieceCommand() {
    return this.runOnce(
        () -> {
          if (io_.game_piece_ == GamePiece.CORAL) {
            setGamePiece(GamePiece.ALGAE);
          } else {
            setGamePiece(GamePiece.CORAL);
          }
        });
  }

  public boolean isCoralMode() {
    return io_.game_piece_ == GamePiece.CORAL;
  }

  public boolean isAlgaeMode() {
    return io_.game_piece_ == GamePiece.ALGAE;
  }

  public boolean hasAlgae() {
    return algae_debouncer_.calculate(
        isAlgaeMode()
            && wheel_motor_.getSupplyCurrent().getValueAsDouble() > 0
            && Util.epislonEquals(wheel_motor_.getVelocity().getValueAsDouble(), 0, 2.5));
  }

  public boolean hasCoral() {
    return coral_debouncer_.calculate(
        isCoralMode()
            && wheel_motor_.getSupplyCurrent().getValueAsDouble()
                > ClawConstants.CORAL_CURRENT_THRESHOLD);
  }

  public ClawMode getClawMode(){
    return io_.claw_mode_;
  }

  public class ClawPeriodicIo implements Logged {
    @Log.File public ClawMode claw_mode_ = ClawMode.IDLE;
    @Log.File public boolean enable_blast_ = false;
    @Log.File public GamePiece game_piece_ = GamePiece.CORAL;
    @Log.File public double wheel_output_ = 0;
    @Log.File public double current_output_ = 0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
