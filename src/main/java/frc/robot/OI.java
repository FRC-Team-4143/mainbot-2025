// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.mw_lib.util.Util;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Elevator.TargetConfig;

public abstract class OI {

  // Sets up both controllers
  static CommandXboxController driver_controller_ = new CommandXboxController(0);
  static CommandXboxController operator_controller_ = new CommandXboxController(1);

  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  static Claw claw_ = Claw.getInstance();
  static Elevator elevator_ = Elevator.getInstance();

  public static void configureBindings() {

    SmartDashboard.putData(
        "Set Wheel Offsets",
        Commands.runOnce(() -> swerve_drivetrain_.tareEverything()).ignoringDisable(true));
    SmartDashboard.putData(
        "Seed Field Centric",
        Commands.runOnce(
                () ->
                    swerve_drivetrain_.seedFieldRelative(swerve_drivetrain_.getDriverPrespective()))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Zero Elevator & Arm",
        Commands.runOnce(() -> elevator_.elevatorAndArmPoseReset()).ignoringDisable(true));

    driver_controller_
        .rightStick()
        .onTrue(
            Commands.runOnce(() -> swerve_drivetrain_.toggleFieldCentric(), swerve_drivetrain_));

    driver_controller_
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> claw_.setClawMode(ClawMode.SHOOT),
                () -> claw_.setClawMode(ClawMode.IDLE),
                claw_));
    driver_controller_
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> claw_.setClawMode(ClawMode.LOAD),
                () -> claw_.setClawMode(ClawMode.IDLE),
                claw_));
    driver_controller_
        .y()
        .onTrue(
            Commands.startEnd(
                () -> elevator_.setCurrentTargetConfig(TargetConfig.L4),
                () -> elevator_.setCurrentTargetConfig(TargetConfig.SOURCE),
                elevator_));
    driver_controller_
        .x()
        .onTrue(
            Commands.startEnd(
                () -> elevator_.setCurrentTargetConfig(TargetConfig.L3),
                () -> elevator_.setCurrentTargetConfig(TargetConfig.SOURCE),
                elevator_));
    driver_controller_
        .b()
        .onTrue(
            Commands.startEnd(
                () -> elevator_.setCurrentTargetConfig(TargetConfig.L2),
                () -> elevator_.setCurrentTargetConfig(TargetConfig.SOURCE),
                elevator_));
    driver_controller_
        .a()
        .onTrue(
            Commands.startEnd(
                () -> elevator_.setCurrentTargetConfig(TargetConfig.L1),
                () -> elevator_.setCurrentTargetConfig(TargetConfig.SOURCE),
                elevator_));
  }

  public static double getDriverJoystickLeftX() {
    double val = driver_controller_.getLeftX();
    double output = val * val;
    output = Math.copySign(output, val);
    // return output;
    return val;
  }

  public static double getDriverJoystickLeftY() {
    double val = driver_controller_.getLeftY();
    double output = val * val;
    output = Math.copySign(output, val);
    // return output;
    return val;
  }

  public static double getDriverJoystickRightX() {
    double val = driver_controller_.getRightX();
    double output = val * val;
    output = Math.copySign(output, val);
    // return output;
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
