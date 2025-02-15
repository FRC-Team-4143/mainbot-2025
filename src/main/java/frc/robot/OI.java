// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaeLoad;
import frc.robot.commands.CoralEject;
import frc.robot.commands.CoralLoad;
import frc.robot.commands.ElevatorButton;
import frc.robot.commands.ElevatorButton.Level;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);
  private static CommandXboxController operator_controller_ = new CommandXboxController(1);

  private static Trigger driver_pov_active_ = new Trigger(getDriverJoystickPOV()::isPresent);

  public static void configureBindings() {

    /*
     *
     * Smart Dashboard Bindings
     *
     */

    // Set Wheel Offsets
    SmartDashboard.putData(
        "Set Wheel Offsets",
        Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything())
            .ignoringDisable(true));
    // Seed Field Centric Forward Direction
    SmartDashboard.putData(
        "Seed Field Centric",
        SwerveDrivetrain.getInstance().seedFieldRelativeCommand().ignoringDisable(true));
    SmartDashboard.putData(
        "Disturb Pose",
        Commands.runOnce(() -> PoseEstimator.getInstance().disturbPose()).ignoringDisable(true));
    // Sync Elevator and Arm Sensor to "Home" Position
    SmartDashboard.putData(
        "Zero Elevator & Arm",
        Commands.runOnce(() -> Elevator.getInstance().elevatorAndArmPoseReset())
            .ignoringDisable(true));

    // Swap Between Robot Centric and Field Centric
    driver_controller_
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () -> SwerveDrivetrain.getInstance().toggleFieldCentric(),
                    SwerveDrivetrain.getInstance())
                .ignoringDisable(true));

    /*
     *
     * Manual Teleop Bindings
     *
     */

    driver_controller_.rightBumper().onTrue(Claw.getInstance().toggleGamePiece());
    driver_controller_
        .leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                new CoralLoad(), new AlgaeLoad(), Claw.getInstance()::isCoralMode));
    driver_controller_
        .rightTrigger()
        .whileTrue(
            new ConditionalCommand(
                new CoralEject(), new AlgaeEject(), Claw.getInstance()::isCoralMode));
    driver_controller_.y().toggleOnTrue(new ElevatorButton(Level.L4));
    driver_controller_.x().toggleOnTrue(new ElevatorButton(Level.L2));
    driver_controller_.b().toggleOnTrue(new ElevatorButton(Level.L3));
    driver_controller_.a().toggleOnTrue(new ElevatorButton(Level.L1));

    // new ConditionalCommand(new SetReefLevel(ReefLevel.L3), new
    // SetReefLevel(ReefLevel.ALGAE_HIGH), Claw.getInstance()::isCoralMode))

    /*
     *
     * Game State Manager Bindings
     *
     */
    // operator_controller_
    // .y()
    // .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L4)));
    // operator_controller_
    // .x()
    // .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L2)));
    // operator_controller_
    // .b()
    // .toggleOnTrue(Commands.runOnce(() ->
    // GameStateManager.wantedTarget(ScoringTarget.REEF_L3)));
    // operator_controller_.a().toggleOnTrue(());
    // operator_controller_.povDown().toggleOnTrue(new
    // SetReefLevel(ReefLevel.ALGAE_LOW));
    // operator_controller_.povUp().toggleOnTrue(new
    // SetReefLevel(ReefLevel.ALGAE_HIGH));

    /*
     *
     * L2 Game State Manager Bindings
     *
     */

    driver_controller_
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  GameStateManager.getInstance().setTargetColumn(Column.RIGHT);
                  GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
                },
                () -> GameStateManager.getInstance().setRobotState(RobotState.END)));
    driver_controller_
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  GameStateManager.getInstance().setTargetColumn(Column.LEFT);
                  GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
                },
                () -> GameStateManager.getInstance().setRobotState(RobotState.END)));

    driver_pov_active_.whileTrue(
        Commands.startEnd(
            () -> SwerveDrivetrain.getInstance().setDriveMode(DriveMode.CRAWL),
            () -> SwerveDrivetrain.getInstance().restoreDefaultDriveMode()));
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
