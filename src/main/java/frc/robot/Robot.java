// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.FieldRegions;
import frc.mw_lib.auto.Auto;
import frc.mw_lib.auto.AutoManager;
import frc.mw_lib.logging.Elastic;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.autos.*;
import frc.robot.commands.L4Hang;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gsm.GameStateManager;
import frc.robot.subsystems.gsm.GameStateManager.RobotState;
import frc.robot.subsystems.drive.SwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveDrivetrain.DriveMode;
import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer robot_container_;
  private Alliance alliance_ = Alliance.Blue;

  Robot(){
    Logger.recordMetadata("PROJECT_NAME", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GIT_DATE", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GIT_BRANCH", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("BUILD_DATE", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("DIRTY", (BuildConstants.DIRTY == 1)? "Uncommitted Changes :(": "All Changes Committed :)");

    // Set up data receivers & replay source
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    robot_container_ = RobotContainer.getInstance();
  }

  @Override
  public void robotInit() {
    OI.configureBindings();
    FieldRegions.makeRegions();
    ProxyServer.configureServer();

    AutoManager.getInstance().registerAutos(new Right_3_Piece());
    AutoManager.getInstance().registerAutos(new Left_3_Piece());

    SmartDashboard.putData(
        "Snapshot", Commands.runOnce(() -> ProxyServer.snapshot("Test Snapshot")));
    SmartDashboard.putData("Sync Match Data", Commands.runOnce(() -> ProxyServer.syncMatchData()));
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    robot_container_.outputTelemetry();

    // updates data from chassis proxy server
    ProxyServer.updateData();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putString("Intake Preference", OI.intake_preference.toString());
  }

  @Override
  public void disabledInit() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.IDLE);
    if (GameStateManager.getInstance().getRobotState() != RobotState.END) {
      GameStateManager.getInstance().setRobotState(RobotState.END);
    }
  }

  @Override
  public void disabledPeriodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Drives Station is Connected
    if (alliance.isPresent()) {
      // Alliance Has Changed
      if (alliance.get() != alliance_) {
        alliance_ = alliance.get();
        // Update Driver Perspective
        SwerveDrivetrain.getInstance()
            .setDriverPerspective(
                alliance_ == Alliance.Red
                    ? SwerveDrivetrain.getInstance().RED_ALLIANCE_HEADING
                    : SwerveDrivetrain.getInstance().BLUE_ALLIANCE_HEADING);
        // Flip Field Regions
        FieldRegions.flipRegions();
      }
    }
  }

  @Override
  public void autonomousInit() {
    Elastic.selectTab("Auto");
    Auto auto = AutoManager.getInstance().getSelectedAuto();
    CommandScheduler.getInstance().schedule(auto);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    AutoManager.getInstance().removeDisplayedAuto();
  }

  @Override
  public void teleopInit() {
    ProxyServer.syncMatchData();
    SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    CommandScheduler.getInstance().cancelAll();
    if (Elevator.getInstance().isElevatorAndArmHung()) {
      CommandScheduler.getInstance()
          .schedule(new L4Hang().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
