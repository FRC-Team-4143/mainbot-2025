package frc.mw_lib.controls;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger

public class TalonFXTuner {

  TalonFX motor_;
  TalonFX[] follower_motors_;
  String system_name_ = "TalonFXTuner - ";
  Slot0Configs gains_;

  /**
   * @param motor primary motor to control
   * @param followers a list of TalonFXs to act as followers to primary motor. Note that the follow
   *     command with be StrictFollower so any motor inversion will need to be done before passing
   *     the motor.
   * @param system_name string to represent system on SmartDashboard
   * @apiNote The tuning system expects all other control reequests being sent to be disabled
   */
  public TalonFXTuner(TalonFX motor, TalonFX[] followers, String system_name) {
    motor_ = motor;
    follower_motors_ = followers;
    system_name_ += (system_name + "/");
    gains_ = new Slot0Configs();
    // Load current gains from motor controller to class object
    motor_.getConfigurator().refresh(gains_);
    setupDashboard();
  }

  /**
   * @param motor primary motor to control
   * @param system_name string to represent system on SmartDashboard
   */
  public TalonFXTuner(TalonFX motor, String system_name) {
    this(motor, new TalonFX[] {}, system_name);
  }

  /** Reads all gains from SmartDashboard tuning group and updates motor config */
  private void updateGains() {
    gains_.kP = SmartDashboard.getNumber(system_name_ + "P", gains_.kP);
    gains_.kI = SmartDashboard.getNumber(system_name_ + "I", gains_.kI);
    gains_.kD = SmartDashboard.getNumber(system_name_ + "D", gains_.kD);
    gains_.kS = SmartDashboard.getNumber(system_name_ + "S", gains_.kS);
    gains_.kV = SmartDashboard.getNumber(system_name_ + "V", gains_.kV);
    gains_.kA = SmartDashboard.getNumber(system_name_ + "A", gains_.kA);
    gains_.kG = SmartDashboard.getNumber(system_name_ + "G", gains_.kG);
    motor_.getConfigurator().apply(gains_);
    // Clears control request after gains are updated for safety
    clearSetpoint();
  }

  /**
   * Updates the control request being applied to the motor
   *
   * @param request supports any ctre control request type, but is intened for ClosedLoop types
   * @return command that set the setpoint when onTrue and clears the setpoint when interrupted
   */
  private Command updateSetpoint(ControlRequest request) {
    return Commands.startEnd(
        () -> {
          motor_.setControl(request);
          // Set all follower motors to same command as leader motor
          for (TalonFX follower : follower_motors_) {
            // follower motors use their own inversion config
            follower.setControl(new StrictFollower(motor_.getDeviceID()));
          }
        },
        () -> clearSetpoint());
  }

  /**
   * Binds the given trigger to execute a setpoint update of type request
   *
   * @param request supports any ctre control request type, but is intened for ClosedLoop types
   * @param trigger wpilib trigger to start/stop the execution of the setpoint command
  */
  public void bindSetpoint(ControlRequest request, Trigger trigger) {
    trigger.whileTrue(updateSetpoint(request));
  }

  /** Set the active motor request to 0 voltage to act as quick disable */
  private void clearSetpoint() {
    motor_.setControl(new VoltageOut(0));
  }

  /** Adds all tunable values to SmartDashboard under the system_name */
  private void setupDashboard() {
    SmartDashboard.putNumber(system_name_ + "P", gains_.kP);
    SmartDashboard.putNumber(system_name_ + "I", gains_.kI);
    SmartDashboard.putNumber(system_name_ + "D", gains_.kD);
    SmartDashboard.putNumber(system_name_ + "S", gains_.kS);
    SmartDashboard.putNumber(system_name_ + "V", gains_.kV);
    SmartDashboard.putNumber(system_name_ + "A", gains_.kA);
    SmartDashboard.putNumber(system_name_ + "G", gains_.kG);
    SmartDashboard.putData(system_name_ + "Update", Commands.runOnce(() -> updateGains()));
  }
}
