// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.ElevatorL1Target;
import frc.robot.commands.ElevatorL2Target;
import frc.robot.commands.ElevatorL3Target;
import frc.robot.commands.ElevatorL4Target;
import frc.robot.commands.GMSTargetLeft;
import frc.robot.commands.GMSTargetRight;
import frc.robot.commands.GamePieceEject;
import frc.robot.commands.GamePieceLoad;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.OffsetType;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);
  private static CommandXboxController operator_controller_ = new CommandXboxController(1);

  private static BooleanSupplier pov_is_present_ = () -> getDriverJoystickPOV().isPresent();
  private static Trigger driver_pov_active_ = new Trigger(pov_is_present_);
  public static BooleanSupplier use_vision =
      () -> SmartDashboard.getBoolean("Vision/Use Vision Features", true);

  public static void configureBindings() {
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

    /*
     *
     * Smart Dashboard Bindings
     *
     */
    // Set Wheel Offsets
    SmartDashboard.putData(
        "Commands/Set Wheel Offsets",
        Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything())
            .ignoringDisable(true));
    // Seed Field Centric Forward Direction
    SmartDashboard.putData(
        "Commands/Seed Field Centric",
        SwerveDrivetrain.getInstance().seedFieldRelativeCommand().ignoringDisable(true));
    SmartDashboard.putData(
        "Commands/Disturb Pose",
        Commands.runOnce(() -> PoseEstimator.getInstance().disturbPose()).ignoringDisable(true));
    SmartDashboard.putBoolean("Vision/Use Vision Features", true);

    /*
     *
     * Driver Controller Bindings
     *
     */

    // Driver Load
    driver_controller_
        .rightBumper()
        .whileTrue(new GamePieceLoad().unless(Climber.getInstance()::lockOutControl));
    // Driver Score
    driver_controller_
        .rightTrigger()
        .whileTrue(new GamePieceEject().unless(Climber.getInstance()::lockOutControl));
    // Robot Align
    driver_controller_
        .leftTrigger()
        .whileTrue(
            new AlignWithTarget().onlyIf(use_vision).unless(Climber.getInstance()::lockOutControl));
    // Toggle Game Piece
    driver_controller_.leftBumper().onTrue(Claw.getInstance().toggleGamePieceCommand());

    if (Climber.getInstance().isEnabled()) {
      // Increment Climb Sequence
      driver_controller_.start().onTrue(Commands.runOnce(() -> Climber.getInstance().nextStage()));
      // Decrement Climb Sequence
      driver_controller_.back().onTrue(Commands.runOnce(() -> Climber.getInstance().backStage()));
    }

    // Swap Between Robot Centric and Field Centric
    driver_controller_
        .rightStick()
        .onTrue(SwerveDrivetrain.getInstance().toggleFieldCentric().ignoringDisable(true));

    // Crawl
    driver_pov_active_.whileTrue(
        Commands.startEnd(
            () -> SwerveDrivetrain.getInstance().setDriveMode(DriveMode.CRAWL),
            () -> SwerveDrivetrain.getInstance().restoreDefaultDriveMode()));

    /*
     *
     * Operator Controller Bindings
     *
     */
    // Set L4 Target:
    // - Algae Mode (Manual) -> Barge
    // - Coral Mode (Manual) -> L4
    // - Any Mode   (Vision) -> Set GSM L4
    operator_controller_
        .y()
        .toggleOnTrue(
            new ElevatorL4Target()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));
    // Set L3 Target:
    // - Algae Mode (Manual) -> Algae High
    // - Coral Mode (Manual) -> L3
    // - Any Mode   (Vision) -> Set GSM L3
    operator_controller_
        .b()
        .toggleOnTrue(
            new ElevatorL3Target()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));
    // Set L2 Target:
    // - Algae Mode (Manual) -> Algae Low
    // - Coral Mode (Manual) -> L2
    // - Any Mode   (Vision) -> Set GSM L2
    operator_controller_
        .x()
        .toggleOnTrue(
            new ElevatorL2Target()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));
    // Set L1 Target:
    // - Algae Mode (Manual) -> Processor
    // - Coral Mode (Manual) -> L1
    // - Any Mode   (Vision) -> Set GSM L1
    operator_controller_
        .a()
        .toggleOnTrue(
            new ElevatorL1Target()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));

    // Set GSM Target Column Left
    operator_controller_
        .leftBumper()
        .onTrue(
            new GMSTargetLeft()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));

    // Set GSM Target Column Right
    operator_controller_
        .rightBumper()
        .onTrue(
            new GMSTargetRight()
                .unless(Climber.getInstance()::lockOutControl)
                .ignoringDisable(true));

    // Manual Adjust Elevator Setpoint Up
    operator_controller_
        .povUp()
        .onTrue(
            Commands.runOnce(() -> Elevator.getInstance().setOffset(OffsetType.ELEVATOR_UP))
                .ignoringDisable(true));
    // Manual Adjust Elevator Setpoint Down
    operator_controller_
        .povDown()
        .onTrue(
            Commands.runOnce(() -> Elevator.getInstance().setOffset(OffsetType.ELEVATOR_DOWN))
                .ignoringDisable(true));
    // Manual Adjust Arm Setpoint Counter Clockwise
    operator_controller_
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> Elevator.getInstance().setOffset(OffsetType.ARM_CCW))
                .ignoringDisable(true));
    // Manual Adjust Arm Setpoint Clockwise
    operator_controller_
        .povRight()
        .onTrue(
            Commands.runOnce(() -> Elevator.getInstance().setOffset(OffsetType.ARM_CW))
                .ignoringDisable(true));

    operator_controller_
        .start()
        .onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Vision/Use Vision Features", true))
                .ignoringDisable(true));
    operator_controller_
        .back()
        .onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Vision/Use Vision Features", false))
                .ignoringDisable(true));
  }

  /**
   * @return driver controller left joystick x axis scaled quadratically
   */
  public static double getDriverJoystickLeftX() {
    double val = driver_controller_.getLeftX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  /**
   * @return driver controller left joystick y axis scaled quadratically
   */
  public static double getDriverJoystickLeftY() {
    double val = driver_controller_.getLeftY();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  /**
   * @return driver controller right joystick x axis scaled quadratically
   */
  public static double getDriverJoystickRightX() {
    double val = driver_controller_.getRightX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  /**
   * @return driver controller joystick pov angle in degs. empty if nothing is pressed
   */
  public static Optional<Rotation2d> getDriverJoystickPOV() {
    int pov = driver_controller_.getHID().getPOV();
    return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
  }

  /*
   *
   * The OI methods below are used for the TalonFX Tuner Bindings.
   * These should not be used in teleop robot control.
   *
   */

  /**
   * @return event trigger bound to driver controller A button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickAButtonTrigger() {
    return driver_controller_.a();
  }

  /**
   * @return event trigger bound to driver controller B button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickBButtonTrigger() {
    return driver_controller_.b();
  }

  /**
   * @return event trigger bound to driver controller Y button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickYButtonTrigger() {
    return driver_controller_.y();
  }

  /**
   * @return event trigger bound to driver controller X button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickXButtonTrigger() {
    return driver_controller_.x();
  }

  /**
   * @return event trigger bound to operator controller A button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickAButtonTrigger() {
    return operator_controller_.a();
  }

  /**
   * @return event trigger bound to driver controller B button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickBButtonTrigger() {
    return operator_controller_.b();
  }

  /**
   * @return event trigger bound to driver controller X button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickYButtonTrigger() {
    return operator_controller_.y();
  }

  /**
   * @return event trigger bound to driver controller Y button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickXButtonTrigger() {
    return operator_controller_.x();
  }
}
