package frc.mw_lib.controls;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TalonFXTuner {

  TalonFX motor_;
  TalonFX[] follower_motors_;
  String system_name_;
  Slot0Configs gains_;

  public TalonFXTuner(TalonFX motor, TalonFX[] followers, String system_name) {
    motor_ = motor;
    follower_motors_ = followers;
    system_name_ = system_name;
    gains_ = new Slot0Configs();
    // Load current gains from motor controller to class object
    motor_.getConfigurator().refresh(gains_);
    setupDashboard();
  }

  /** Reads all gains from SmartDashboard tuning group and updates motor config */
  private void updateGains() {
    gains_.kP = SmartDashboard.getNumber(system_name_ + " - P", gains_.kP);
    gains_.kI = SmartDashboard.getNumber(system_name_ + " - I", gains_.kI);
    gains_.kD = SmartDashboard.getNumber(system_name_ + " - D", gains_.kD);
    gains_.kS = SmartDashboard.getNumber(system_name_ + " - S", gains_.kS);
    gains_.kV = SmartDashboard.getNumber(system_name_ + " - V", gains_.kV);
    gains_.kA = SmartDashboard.getNumber(system_name_ + " - A", gains_.kA);
    gains_.kG = SmartDashboard.getNumber(system_name_ + " - G", gains_.kG);
    motor_.getConfigurator().apply(gains_);
    // Clears control request after gains are updated for safety
    clearSetpoint();
  }

  public Command updateSetpoint(ControlRequest request) {
    return Commands.startEnd(
        () -> {
          motor_.setControl(request);
          for (TalonFX follower : follower_motors_) {
            follower.setControl(new Follower(motor_.getDeviceID(), false));
          }
        },
        () -> clearSetpoint());
  }

  private void clearSetpoint() {
    motor_.setControl(new VoltageOut(0));
  }

  private void setupDashboard() {
    SmartDashboard.putNumber(system_name_ + " - P", gains_.kP);
    SmartDashboard.putNumber(system_name_ + " - I", gains_.kI);
    SmartDashboard.putNumber(system_name_ + " - D", gains_.kD);
    SmartDashboard.putNumber(system_name_ + " - S", gains_.kS);
    SmartDashboard.putNumber(system_name_ + " - V", gains_.kV);
    SmartDashboard.putNumber(system_name_ + " - A", gains_.kA);
    SmartDashboard.putNumber(system_name_ + " - G", gains_.kG);
    SmartDashboard.putData(system_name_ + " - Update", Commands.runOnce(() -> updateGains()));
  }
}
