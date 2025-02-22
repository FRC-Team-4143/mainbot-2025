// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FieldRegions;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;

public class CoralStation extends Command {

  static Elevator elevator_;

  /** Creates a new CoralStationLoad. */
  public CoralStation() {
    elevator_ = Elevator.getInstance();
    addRequirements(elevator_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_.setSpeedLimit(SpeedLimit.CORAL);
    elevator_.setTarget(Constants.ElevatorConstants.Target.STATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if in zone
    if(FieldRegions.STATION_REGIONS[1].contains(PoseEstimator.getInstance().getRobotPose())){
      Rotation2d rightTarget = FieldRegions.REGION_POSE_TABLE.get(FieldRegions.STATION_REGIONS[1].getName()).getRotation(); 
      SwerveDrivetrain.getInstance().setTargetRotation(rightTarget);
    }
    else if(FieldRegions.STATION_REGIONS[0].contains(PoseEstimator.getInstance().getRobotPose())){
      Rotation2d leftTarget = FieldRegions.REGION_POSE_TABLE.get(FieldRegions.STATION_REGIONS[0].getName()).getRotation();
      SwerveDrivetrain.getInstance().setTargetRotation(leftTarget);
    }
    else{
      SwerveDrivetrain.getInstance().restoreDefaultDriveMode(); 
    }
    //then pull tag rotation

    //then set target angle based on rotation

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_.stowElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
