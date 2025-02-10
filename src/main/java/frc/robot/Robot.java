// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FieldRegions;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;

public class Robot extends TimedRobot {
  private RobotContainer robot_container_;
  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  private Alliance allaince_ = Alliance.Blue;
  private GameStateManager game_state_manager = new GameStateManager();

  @Override
  public void robotInit() {
    robot_container_ = RobotContainer.getInstance();
    AutoManager.getInstance();
    OI.configureBindings();
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    robot_container_.outputTelemetry();
  }

  @Override
  public void disabledInit() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.IDLE);
    FieldRegions.constructRegions(false);
  }

  @Override
  public void disabledPeriodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Drives Station is Connected
    if (alliance.isPresent()) {
      // Alliance Has Changed
      if (alliance.get() != allaince_) {
        allaince_ = alliance.get();
        // Update Driver Prespective
        swerve_drivetrain_.setDriverPerspective(
            allaince_ == Alliance.Red
                ? swerve_drivetrain_.RED_ALLIANCE_HEADING
                : swerve_drivetrain_.BLUE_ALLIANCE_HEADING);
        // Update Field Regions
        FieldRegions.constructRegions(true);
      }
    }
  }

  @Override
  public void autonomousInit() {
    swerve_drivetrain_.setDriveMode(DriveMode.TRAJECTORY);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    swerve_drivetrain_.restoreDefaultDriveMode();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
    game_state_manager.updateGameState();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
