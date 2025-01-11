// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.Feed;
import frc.robot.commands.Score;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CoralFunnel.FeedingMode;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import frc.lib.Util;

public abstract class OI {

    // Sets up both controllers
    static CommandXboxController driver_controller_ = new CommandXboxController(0);

    static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
    static CoralFunnel coral_funnel_ = CoralFunnel.getInstance();

    public static void configureBindings() {

        SmartDashboard.putData("Set Wheel Offsets", Commands.runOnce(
                () -> swerve_drivetrain_.tareEverything())
                .ignoringDisable(true));
        SmartDashboard.putData("Seed Field Centric", Commands.runOnce(
                () -> swerve_drivetrain_.seedFieldRelative(swerve_drivetrain_.getDriverPrespective()))
                .ignoringDisable(true));

        driver_controller_.rightStick().onTrue(Commands.runOnce(() -> swerve_drivetrain_.toggleFieldCentric(), swerve_drivetrain_));

        driver_controller_.leftTrigger().whileTrue(new Feed());

        driver_controller_.rightTrigger().whileTrue(new Score());
    }

    static public double getDriverJoystickLeftX() {
        double val = driver_controller_.getLeftX();
        double output = val * val;
        output = Math.copySign(output, val);
        //return output;
        return val;
    }

    static public double getDriverJoystickLeftY() {
        double val = driver_controller_.getLeftY();
        double output = val * val;
        output = Math.copySign(output, val);
        //return output;
        return val;
    }

    static public double getDriverJoystickRightX() {
        double val = driver_controller_.getRightX();
        double output = val * val;
        output = Math.copySign(output, val);
        //return output;
        return val;
    }

    static public boolean getDriverJoystickRightY() {
        double val = driver_controller_.getRightY();
        return Util.epislonEquals(val, 0, 0.1);
    }

    static public double getDriverJoystickPOVangle() {
        return driver_controller_.getHID().getPOV();
    }

}
