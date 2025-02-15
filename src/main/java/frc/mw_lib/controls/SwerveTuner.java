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

public class SwerveTuner {

  TalonFX[] drive_motors_;
  TalonFX[] steer_motors_;
  String system_name_ = "SwerveTuner - ";
  Slot0Configs drive_config_;
  Slot0Configs steer_config_;
  SysIdRoutine routine_;

  /**
   * @param motor primary motor to control
   * @param followers a list of TalonFXs to act as followers to primary motor. Note that the follow
   *     command with be StrictFollower so any motor inversion will need to be done before passing
   *     the motor.
   * @param system_name string to represent system on SmartDashboard
   * @apiNote The tuning system expects all other control reequests being sent to be disabled
   */
  public SwerveTuner(TalonFX[] drive, TalonFX[] steer, String system_name, Subsystem subsystem) {
    drive_motors_ = drive;
    steer_motors_ = steer;
    system_name_ += (system_name + "/");

    routine_ =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  for (int i = 0; i < drive_motors_.length; i++) {
                    drive_motors_[i].setControl(new VoltageOut(voltage));
                  }
                  for (int i = 0; i < steer_motors_.length; i++) {
                    steer_motors_[i].setControl(new VoltageOut(voltage));
                  }
                },
                log -> {
                  for (int i = 0; i < drive_motors_.length; i++) {
                    log.motor(system_name_)
                      .voltage(drive_motors_[i].getMotorVoltage().getValue())
                      .angularPosition(drive_motors_[i].getPosition().getValue())
                      .angularVelocity(drive_motors_[i].getVelocity().getValue());
                  }
                  for (int i = 0; i < steer_motors_.length; i++) {
                    log.motor(system_name_)
                      .voltage(steer_motors_[i].getMotorVoltage().getValue())
                      .angularPosition(steer_motors_[i].getPosition().getValue())
                      .angularVelocity(steer_motors_[i].getVelocity().getValue());
                  }
                },
                subsystem));

    setupDashboard();
  }

  /**
   * @param motor primary motor to control
   * @param system_name string to represent system on SmartDashboard
   */
  public SwerveTuner( String system_name, Subsystem subsystem) {
    this(new TalonFX[] {}, new TalonFX[] {}, system_name, subsystem);
  }

  /** Reads all gains from SmartDashboard tuning group and updates motor config */
  private void updateGains() {
    drive_config_.kP = SmartDashboard.getNumber(system_name_ + "P-Drive", 0);
    drive_config_.kI = SmartDashboard.getNumber(system_name_ + "I-Drive", 0);
    drive_config_.kD = SmartDashboard.getNumber(system_name_ + "D-Drive", 0);
    drive_config_.kS = SmartDashboard.getNumber(system_name_ + "S-Drive", 0);
    drive_config_.kV = SmartDashboard.getNumber(system_name_ + "V-Drive", 0);
    drive_config_.kA = SmartDashboard.getNumber(system_name_ + "A-Drive", 0);
    drive_config_.kG = SmartDashboard.getNumber(system_name_ + "G-Drive", 0);

    steer_config_.kP = SmartDashboard.getNumber(system_name_ + "P-Steer", 0);
    steer_config_.kI = SmartDashboard.getNumber(system_name_ + "I-Steer", 0);
    steer_config_.kD = SmartDashboard.getNumber(system_name_ + "D-Steer", 0);
    steer_config_.kS = SmartDashboard.getNumber(system_name_ + "S-Steer", 0);
    steer_config_.kV = SmartDashboard.getNumber(system_name_ + "V-Steer", 0);
    steer_config_.kA = SmartDashboard.getNumber(system_name_ + "A-Steer", 0);
    steer_config_.kG = SmartDashboard.getNumber(system_name_ + "G-Steer", 0);

    for(int i = 0; i < drive_motors_.length; i++){

      DataLogManager.log(
        "Motor ID: " + drive_motors_[i].getDeviceID() + " - Updating Gains" + drive_config_.toString());

    }

    for(int i = 0; i < steer_motors_.length; i++){

      DataLogManager.log(
        "Motor ID: " + steer_motors_[i].getDeviceID() + " - Updating Gains" + steer_config_.toString());

    }

    StatusCode status;

    for(int i = 0; i < drive_motors_.length; i++){

      status = drive_motors_[i].getConfigurator().apply(drive_config_);

      if (!status.isOK()) {
        DataLogManager.log(
            "Motor ID: "
                + drive_motors_[i].getDeviceID()
                + " - Error Updating Gains: "
                + status.getDescription());
      }

    }

    for(int i = 0; i < steer_motors_.length; i++){

      status = steer_motors_[i].getConfigurator().apply(steer_config_);

      if (!status.isOK()) {
        DataLogManager.log(
            "Motor ID: "
                + steer_motors_[i].getDeviceID()
                + " - Error Updating Gains: "
                + status.getDescription());
      }

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
  private Command updateSetpointDrive(ControlRequest request) {
    return new FunctionalCommand(
            // Set motor requests
            () -> {
              for (TalonFX drive : drive_motors_) {
                // follower motors use their own inversion config
                drive.setControl(request);
                DataLogManager.log("Motor ID: " + drive.getDeviceID() + " - " + request.toString());
              }
            },
            // Put Closed Loop Data On Dashboard
            () -> {
              for (TalonFX drive : drive_motors_) {
                // follower motors use their own inversion config
                SmartDashboard.putNumber(
                  system_name_ + "Setpoint-Drive", drive.getClosedLoopReference().getValue());
                SmartDashboard.putNumber(
                  system_name_ + "Feedback-Drive",
                  drive.getClosedLoopReference().getValue()
                      - drive.getClosedLoopError().getValue());
                SmartDashboard.putNumber(
                  system_name_ + "Error-Drive", drive.getClosedLoopError().getValue());
              }
            },
            // Clear the active request and setpoint
            (interrupted) -> clearSetpoint(),
            () -> false)
        .onlyIf(RobotState::isTest);
  }

  private Command updateSetpointSteer(ControlRequest request) {
    return new FunctionalCommand(
            // Set motor requests
            () -> {
              for (TalonFX steer : steer_motors_) {
                // follower motors use their own inversion config
                steer.setControl(request);
                DataLogManager.log("Motor ID: " + steer.getDeviceID() + " - " + request.toString());
              }
            },
            // Put Closed Loop Data On Dashboard
            () -> {
              for (TalonFX steer : steer_motors_) {
                // follower motors use their own inversion config
                SmartDashboard.putNumber(
                  system_name_ + "Setpoint-Steer", steer.getClosedLoopReference().getValue());
                SmartDashboard.putNumber(
                  system_name_ + "Feedback-Steer",
                  steer.getClosedLoopReference().getValue()
                      - steer.getClosedLoopError().getValue());
                SmartDashboard.putNumber(
                  system_name_ + "Error", steer.getClosedLoopError().getValue());
              }
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
  public void bindSetpointDrive(ControlRequest request, Trigger trigger) {
    trigger.whileTrue(updateSetpointDrive(request));
  }

  public void bindSetpointSteer(ControlRequest request, Trigger trigger) {
    trigger.whileTrue(updateSetpointSteer(request));
  }

  /** Set the active motor request to 0 voltage to act as quick disable */
  private void clearSetpoint() {
    for(TalonFX drive : drive_motors_){
      drive.setControl(new VoltageOut(0));
      DataLogManager.log("Motor ID: " + drive.getDeviceID() + " - Disabled");
    }
    for(TalonFX steer : steer_motors_){
      steer.setControl(new VoltageOut(0));
      DataLogManager.log("Motor ID: " + steer.getDeviceID() + " - Disabled");
    }
  }

  /** Adds all tunable values to SmartDashboard under the system_name */
  private void setupDashboard() {
    SmartDashboard.putNumber(system_name_ + "P-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "I-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "D-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "S-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "V-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "A-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "G-Drive", 0);

    SmartDashboard.putNumber(system_name_ + "P-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "I-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "D-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "S-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "V-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "A-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "G-Steer", 0);
    SmartDashboard.putData(
        system_name_ + "Update",
        Commands.runOnce(() -> updateGains()).onlyIf(RobotState::isTest).ignoringDisable(true));
    SmartDashboard.putNumber(system_name_ + "Setpoint-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "Setpoint-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "Feedback-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "Feedback-Steer", 0);
    SmartDashboard.putNumber(system_name_ + "Error-Drive", 0);
    SmartDashboard.putNumber(system_name_ + "Error-Steer", 0);
    for (TalonFX drive : drive_motors_){

      SmartDashboard.putData(
        system_name_ + "Zero",
        Commands.runOnce(() -> drive.setPosition(0))
            .onlyIf(RobotState::isTest)
            .ignoringDisable(true));

    }
    for (TalonFX steer : steer_motors_){

      SmartDashboard.putData(
        system_name_ + "Zero",
        Commands.runOnce(() -> steer.setPosition(0))
            .onlyIf(RobotState::isTest)
            .ignoringDisable(true));

    }
    
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
