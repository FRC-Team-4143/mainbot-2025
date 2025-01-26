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

public abstract class OI {

  // Sets up both controllers
  static CommandXboxController driver_controller_ = new CommandXboxController(0);

  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  static CoralFunnel coral_funnel_ = CoralFunnel.getInstance();

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
