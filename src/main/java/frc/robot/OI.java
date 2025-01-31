// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.mw_lib.util.Util;
import frc.robot.commands.Feed;
import frc.robot.commands.Score;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Claw.ClawMode;

public abstract class OI {

  // Sets up both controllers
  static CommandXboxController driver_controller_ = new CommandXboxController(0);

  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  static Claw claw_ = Claw.getInstance();

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

    driver_controller_
        .rightStick()
        .onTrue(
            Commands.runOnce(() -> swerve_drivetrain_.toggleFieldCentric(), swerve_drivetrain_));

    driver_controller_.leftTrigger().whileTrue(new Feed());

    driver_controller_.rightTrigger().whileTrue(new Score());
    driver_controller_
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> claw_.setClawMode(ClawMode.SHOOT),
                () -> claw_.setClawMode(ClawMode.IDLE),
                claw_));
    driver_controller_
        .y()
        .whileTrue(
            Commands.startEnd(
                () -> claw_.setClawMode(ClawMode.LOAD),
                () -> claw_.setClawMode(ClawMode.CLOSED),
                claw_));
    driver_controller_
        .b()
        .whileTrue(
            Commands.startEnd(
                () -> claw_.setClawMode(ClawMode.OPEN),
                () -> claw_.setClawMode(ClawMode.CLOSED),
                claw_));

    /* driver_controller_.x().onTrue(Commands.runOnce(
    () -> {
      var disturbance =
      new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
      drivetrain.resetPose(drivetrain.getPose().plus(disturbance), false);}).ignoringDisable(true));*/
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

  public static double getDriverJoystickPOVangle() {
    return driver_controller_.getHID().getPOV();
  }
}
