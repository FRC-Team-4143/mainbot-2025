// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.PoseEstimator;
import frc.lib.FieldRegions;
import frc.lib.ScoringPoses;
import frc.mw_lib.geometry.Region;
import frc.robot.Constants;

public class ScoreBarge extends Command {

  static Elevator elevator_;
  static SwerveDrivetrain drivetrain_;
  static PoseEstimator poseEstimator_;
  static Claw claw_;
  
  static Optional<Region> current_region = Optional.empty();

  /** Creates a new CoralStationLoad. */
  public ScoreBarge() {
    elevator_ = Elevator.getInstance();
    drivetrain_ = SwerveDrivetrain.getInstance();
    poseEstimator_ = PoseEstimator.getInstance();
    claw_ = Claw.getInstance();
    addRequirements(elevator_);
    addRequirements(drivetrain_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_region = poseEstimator_.algaeRegion();
    if (current_region.isPresent() && current_region.get().getName() == "Barge" && claw_.isAlgaeMode()) {
      drivetrain_.setTightRope(ScoringPoses.BARGE_TIGHT_ROPE);
    } else {
      drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
