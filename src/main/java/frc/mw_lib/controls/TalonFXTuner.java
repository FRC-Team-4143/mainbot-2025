package frc.mw_lib.controls;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TalonFXTuner {

  TalonFX motor_;
  TalonFX[] follower_motors_;
  String system_name_ = "Tuning/TalonFXTuner - ";
  Slot0Configs config_;
  SysIdRoutine routine_;

  /**
   * @param motor primary motor to control
   * @param followers a list of TalonFXs to act as followers to primary motor. Note that the follow
   *     command with be StrictFollower so any motor inversion will need to be done before passing
   *     the motor.
   * @param system_name string to represent system on SmartDashboard
   * @apiNote The tuning system expects all other control reequests being sent to be disabled
   */
  public TalonFXTuner(TalonFX motor, TalonFX[] followers, String system_name, Subsystem subsystem) {
    motor_ = motor;
    follower_motors_ = followers;
    system_name_ += (system_name + "/");
    config_ = new Slot0Configs();
    motor_.getConfigurator().refresh(config_);

    routine_ =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  motor_.setControl(new VoltageOut(voltage));
                  // Set all follower motors to same command as leader motor
                  for (TalonFX follower : follower_motors_) {
                    // follower motors use their own inversion config
                    follower.setControl(new StrictFollower(motor_.getDeviceID()));
                  }
                },
                log -> {
                  log.motor(system_name_)
                      .voltage(motor_.getMotorVoltage().getValue())
                      .angularPosition(motor_.getPosition().getValue())
                      .angularVelocity(motor_.getVelocity().getValue());
                },
                subsystem));

    setupDashboard();
  }

  /**
   * @param motor primary motor to control
   * @param system_name string to represent system on SmartDashboard
   */
  public TalonFXTuner(TalonFX motor, String system_name, Subsystem subsystem) {
    this(motor, new TalonFX[] {}, system_name, subsystem);
  }

  /** Reads all gains from SmartDashboard tuning group and updates motor config */
  private void updateGains() {
    config_.kP = SmartDashboard.getNumber(system_name_ + "P", 0);
    config_.kI = SmartDashboard.getNumber(system_name_ + "I", 0);
    config_.kD = SmartDashboard.getNumber(system_name_ + "D", 0);
    config_.kS = SmartDashboard.getNumber(system_name_ + "S", 0);
    config_.kV = SmartDashboard.getNumber(system_name_ + "V", 0);
    config_.kA = SmartDashboard.getNumber(system_name_ + "A", 0);
    config_.kG = SmartDashboard.getNumber(system_name_ + "G", 0);

    DataLogManager.log(
        "Motor ID: " + motor_.getDeviceID() + " - Updating Gains" + config_.toString());

    StatusCode status = motor_.getConfigurator().apply(config_);
    if (!status.isOK()) {
      DataLogManager.log(
          "Motor ID: "
              + motor_.getDeviceID()
              + " - Error Updating Gains: "
              + status.getDescription());
    }
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
    return new FunctionalCommand(
            // Set motor requests
            () -> {
              motor_.setControl(request);
              DataLogManager.log("Motor ID: " + motor_.getDeviceID() + " - " + request.toString());
              // Set all follower motors to same command as leader motor
              for (TalonFX follower : follower_motors_) {
                // follower motors use their own inversion config
                follower.setControl(new StrictFollower(motor_.getDeviceID()));
              }
            },
            // Put Closed Loop Data On Dashboard
            () -> {
              SmartDashboard.putNumber(
                  system_name_ + "Setpoint", motor_.getClosedLoopReference().getValue());
              SmartDashboard.putNumber(
                  system_name_ + "Feedback",
                  motor_.getClosedLoopReference().getValue()
                      - motor_.getClosedLoopError().getValue());
              SmartDashboard.putNumber(
                  system_name_ + "Error", motor_.getClosedLoopError().getValue());
            },
            // Clear the active request and setpoint
            (interrupted) -> clearSetpoint(),
            () -> false)
        .onlyIf(RobotState::isTest);
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
    DataLogManager.log("Motor ID: " + motor_.getDeviceID() + " - Disabled");
  }

  /** Adds all tunable values to SmartDashboard under the system_name */
  private void setupDashboard() {
    SmartDashboard.putNumber(system_name_ + "P", config_.kP);
    SmartDashboard.putNumber(system_name_ + "I", config_.kI);
    SmartDashboard.putNumber(system_name_ + "D", config_.kD);
    SmartDashboard.putNumber(system_name_ + "S", config_.kS);
    SmartDashboard.putNumber(system_name_ + "V", config_.kV);
    SmartDashboard.putNumber(system_name_ + "A", config_.kA);
    SmartDashboard.putNumber(system_name_ + "G", config_.kG);
    SmartDashboard.putData(
        system_name_ + "Update",
        Commands.runOnce(() -> updateGains()).onlyIf(RobotState::isTest).ignoringDisable(true));
    SmartDashboard.putNumber(system_name_ + "Setpoint", 0);
    SmartDashboard.putNumber(system_name_ + "Feedback", 0);
    SmartDashboard.putNumber(system_name_ + "Error", 0);
    SmartDashboard.putData(
        system_name_ + "Zero",
        Commands.runOnce(() -> motor_.setPosition(0))
            .onlyIf(RobotState::isTest)
            .ignoringDisable(true));
  }

  private SysIdRoutine getSysIdRoutine() {
    return routine_;
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().dynamic(direction).onlyIf(RobotState::isTest);
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().quasistatic(direction).onlyIf(RobotState::isTest);
  }

  public void bindDynamicForward(Trigger trigger) {
    trigger.whileTrue(sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
  }

  public void bindDynamicReverse(Trigger trigger) {
    trigger.whileTrue(sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
  }

  public void bindQuasistaticForward(Trigger trigger) {
    trigger.whileTrue(sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
  }

  public void bindQuasistaticReverse(Trigger trigger) {
    trigger.whileTrue(sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
  }
}
