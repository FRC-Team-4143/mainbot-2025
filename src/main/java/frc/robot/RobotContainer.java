// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.mw_lib.subsystem.SubsystemManager;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral_detector.CoralDetector;
import frc.robot.subsystems.drive.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gsm.GameStateManager;
import frc.robot.subsystems.pickup.Pickup;
import frc.robot.subsystems.pose_estimator.PoseEstimator;

public class RobotContainer extends SubsystemManager {
  private static RobotContainer instance;

  public static synchronized RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    registerSubsystem(SwerveDrivetrain.getInstance());
    registerSubsystem(PoseEstimator.getInstance());
    registerSubsystem(Claw.getInstance());
    registerSubsystem(Elevator.getInstance());
    registerSubsystem(Climber.getInstance());
    registerSubsystem(GameStateManager.getInstance());
    registerSubsystem(Pickup.getInstance());
    registerSubsystem(CoralDetector.getInstance());

    // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
    completeRegistration();
  }
}
