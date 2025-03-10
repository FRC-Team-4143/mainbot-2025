// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.Target;
import frc.lib.FieldRegions;
import frc.lib.ScoringPoses;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.SwerveDrivetrain.SpeedPresets;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;

public class CoralStation extends Command {

  static Elevator elevator_;

  /** Creates a new CoralStationLoad. */
  public CoralStation() {
    elevator_ = Elevator.getInstance();
    addRequirements(elevator_);
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_.setSpeedLimit(SpeedLimit.CORAL);
    elevator_.setTarget(Target.STATION);
    Claw.getInstance().setGamePiece(GamePiece.CORAL);
    Claw.getInstance().setClawMode(ClawMode.LOAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

if (FieldRegions.LEFT_CORAL_STATION_SLOW_REGION.contains(PoseEstimator.getInstance().getRobotPose()) || FieldRegions.RIGHT_CORAL_STATION_SLOW_REGION.contains(PoseEstimator.getInstance().getRobotPose())) {
  SwerveDrivetrain.getInstance().setActiveSpeed(SpeedPresets.ONE_THIRD_SPEED);
}
    // if in zone
    if (FieldRegions.LEFT_CORAL_STATION_REGION.contains(PoseEstimator.getInstance().getRobotPose())) {
      SwerveDrivetrain.getInstance()
          .setTargetRotation(ScoringPoses.LEFT_CORAL_STATION_POSE.getRotation());

    } else if (FieldRegions.RIGHT_CORAL_STATION_REGION.contains(PoseEstimator.getInstance().getRobotPose())) {
      SwerveDrivetrain.getInstance()
          .setTargetRotation(ScoringPoses.RIGHT_CORAL_STATION_POSE.getRotation());

    } else {
      SwerveDrivetrain.getInstance().setActiveSpeed(SpeedPresets.MAX_SPEED);
      SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    }
    // then pull tag rotation

    // then set target angle based on rotation

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    Elevator.getInstance().setTarget(Target.STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
