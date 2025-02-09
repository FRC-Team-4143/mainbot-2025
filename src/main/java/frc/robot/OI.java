// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.mw_lib.util.Util;
import frc.robot.GameStateManager.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public abstract class OI {

  // Sets up both controllers
  static CommandXboxController driver_controller_ = new CommandXboxController(0);
  static PoseEstimator pose_estimator_ = PoseEstimator.getInstance();
  static CommandXboxController operator_controller_ = new CommandXboxController(1);

  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  static Claw claw_ = Claw.getInstance();
  static Elevator elevator_ = Elevator.getInstance();
  private static GameStateManager game_state_manager = new GameStateManager();

  public static void configureBindings() {

    // Set Wheel Offsets
    SmartDashboard.putData(
        "Set Wheel Offsets",
        Commands.runOnce(() -> swerve_drivetrain_.tareEverything()).ignoringDisable(true));
    // Seed Field Centric Forward Direction
    SmartDashboard.putData(
        "Seed Field Centric", swerve_drivetrain_.seedFieldRelativeCommand().ignoringDisable(true));
    SmartDashboard.putData(
        "Disturb Pose",
        Commands.runOnce(() -> pose_estimator_.disturbPose()).ignoringDisable(true));
    // Sync Elevator and Arm Sensor to "Home" Position
    SmartDashboard.putData(
        "Zero Elevator & Arm",
        Commands.runOnce(() -> elevator_.elevatorAndArmPoseReset()).ignoringDisable(true));

    // Swap Between Robot Centric and Field Centric
    driver_controller_
        .rightStick()
        .onTrue(
            Commands.runOnce(() -> swerve_drivetrain_.toggleFieldCentric(), swerve_drivetrain_)
                .ignoringDisable(true));

    // driver_controller_.rightBumper().onTrue(claw_.toggleGamePiece());
    // driver_controller_
    //     .leftTrigger()
    //     .whileTrue(
    //         new ConditionalCommand(new AlgaeLoad(), new CoralStationLoad(), claw_::isAlgaeMode));
    // // Score
    // driver_controller_
    //     .rightTrigger()
    //     .whileTrue(new ConditionalCommand(new CoralEject(), new AlgaeEject(),
    // claw_::isCoralMode));
    // operator_controller_
    //     .y()
    //     .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L4)));
    // operator_controller_
    //     .x()
    //     .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L2)));
    // operator_controller_
    //     .b()
    //     .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L3)));
    // // operator_controller_.a().toggleOnTrue(());
    // operator_controller_.povDown().toggleOnTrue(new SetReefLevel(ReefLevel.ALGAE_LOW));
    // operator_controller_.povUp().toggleOnTrue(new SetReefLevel(ReefLevel.ALGAE_HIGH));
    // // driver_controller_.a().whileTrue(new setReefLevel(ReefLevel.L1)); TODO: Fix Effector
    // // Collision with Frame

    driver_controller_
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> game_state_manager.setRobotState(RobotState.TARGET_ACQUISITION),
                () -> game_state_manager.setRobotState(RobotState.TELEOP_CONTROL)));
  }

  public static double getDriverJoystickLeftX() {
    double val = driver_controller_.getLeftX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  public static double getDriverJoystickLeftY() {
    double val = driver_controller_.getLeftY();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  public static double getDriverJoystickRightX() {
    double val = driver_controller_.getRightX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  public static boolean getDriverJoystickRightY() {
    double val = driver_controller_.getRightY();
    return Util.epislonEquals(val, 0, 0.1);
  }

  public static double getDriverJoystickRightTriggerAxis() {
    return driver_controller_.getRightTriggerAxis();
  }

  public static double getDriverJoystickPOVangle() {
    return driver_controller_.getHID().getPOV();
  }

  public static Trigger getDriverJoystickAButtonTrigger() {
    return driver_controller_.a();
  }

  public static Trigger getDriverJoystickBButtonTrigger() {
    return driver_controller_.b();
  }

  public static Trigger getDriverJoystickYButtonTrigger() {
    return driver_controller_.y();
  }

  public static Trigger getDriverJoystickXButtonTrigger() {
    return driver_controller_.x();
  }

  public static Trigger getOperatorJoystickAButtonTrigger() {
    return operator_controller_.a();
  }

  public static Trigger getOperatorJoystickBButtonTrigger() {
    return operator_controller_.b();
  }

  public static Trigger getOperatorJoystickYButtonTrigger() {
    return operator_controller_.y();
  }

  public static Trigger getOperatorJoystickXButtonTrigger() {
    return operator_controller_.x();
  }
}
