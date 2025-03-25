// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FieldRegions;
import frc.mw_lib.auto.Auto;
import frc.mw_lib.auto.AutoManager;
import frc.mw_lib.logging.Elastic;
import frc.robot.autos.Bump_E4_RightStation_C4_RightStation_D4;
import frc.robot.autos.Bump_J4_LeftStation_L4_LeftStation_K4;
import frc.robot.autos.E4_RightStation_C4;
import frc.robot.autos.E4_RightStation_C4_RightStation_D4;
import frc.robot.autos.H4_Algae;
import frc.robot.autos.J4_LeftStation_L4;
import frc.robot.autos.J4_LeftStation_L4_LeftStation_K4;
import frc.robot.commands.HangProtection;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.RobotState;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;

public class Robot extends TimedRobot {
  private RobotContainer robot_container_;
  private Alliance alliance_ = Alliance.Blue;

  @Override
  public void robotInit() {
    robot_container_ = RobotContainer.getInstance();
    OI.configureBindings();
    FieldRegions.makeRegions();

    AutoManager.getInstance()
        .registerAutos(
            new J4_LeftStation_L4(),
            new J4_LeftStation_L4_LeftStation_K4(),
            new E4_RightStation_C4(),
            new E4_RightStation_C4_RightStation_D4(),
            // new H4_GH_Barge(),
            // new H4_GH_Barge_IJ_Barge()
            new H4_Algae(),
            new Bump_E4_RightStation_C4_RightStation_D4(),
            new Bump_J4_LeftStation_L4_LeftStation_K4());
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    robot_container_.outputTelemetry();

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
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    AutoManager.getInstance().removeDisplayedAuto();
  }

  @Override
  public void teleopInit() {
    SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance()
        .schedule(new HangProtection().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
