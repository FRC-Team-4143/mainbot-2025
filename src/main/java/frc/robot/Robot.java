// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

public class Robot extends TimedRobot {
  private RobotContainer robot_container_;
  private Vision vision;
  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  static PoseEstimator pose_estimator_ = PoseEstimator.getInstance();

  @Override
  public void robotInit() {
    robot_container_ = RobotContainer.getInstance();
    AutoManager.getInstance();
    vision = Vision.getInstance();
    OI.configureBindings();
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
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
  }

  @Override
  public void disabledPeriodic() {
    updateDriverPrespective();
  }

  @Override
  public void autonomousInit() {
    swerve_drivetrain_.setDriveMode(DriveMode.AUTONOMOUS);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private void updateDriverPrespective() {
    if (DriverStation.getAlliance().isPresent()) {
      swerve_drivetrain_.setDriverPrespective(
          DriverStation.getAlliance().get() == Alliance.Red
              ? swerve_drivetrain_.redAlliancePerspectiveRotation
              : swerve_drivetrain_.blueAlliancePerspectiveRotation);
    }
  }
}
