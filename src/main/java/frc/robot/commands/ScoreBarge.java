// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.FieldRegions;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pose_estimator.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrivetrain;

public class ScoreBarge extends Command {

  static Elevator elevator_;
  static SwerveDrivetrain drivetrain_;
  static PoseEstimator poseEstimator_;
  static Claw claw_;

  /** Creates a new CoralStationLoad. */
  public ScoreBarge() {
    elevator_ = Elevator.getInstance();
    drivetrain_ = SwerveDrivetrain.getInstance();
    poseEstimator_ = PoseEstimator.getInstance();
    claw_ = Claw.getInstance();
    addRequirements(elevator_);
    addRequirements(drivetrain_);
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (FieldRegions.BARGE_ENTER.contains(poseEstimator_.getRobotPose())
        || FieldRegions.OPP_BARGE_ENTER.contains(poseEstimator_.getRobotPose())) {
      elevator_.setTarget(TargetType.BARGE);
    } else {
      elevator_.setTarget(TargetType.ALGAE_STOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_.setTarget(TargetType.ALGAE_STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
