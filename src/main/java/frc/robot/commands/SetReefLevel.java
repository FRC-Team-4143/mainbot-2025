// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FieldConstants;
import frc.robot.subsystems.Elevator;

public class SetReefLevel extends Command {

  public enum ReefLevel {
    L1,
    L2,
    L3,
    L4
  }

  private ReefLevel requested_level_;
  private Elevator elevator_;

  public SetReefLevel(ReefLevel level) {
    elevator_ = Elevator.getInstance();
    addRequirements(elevator_);
    requested_level_ = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (requested_level_) {
      case L1:
        elevator_.setEndEffectorHeight(
            FieldConstants.ReefHeight.L1.HEIGHT, FieldConstants.ReefHeight.L1.PITCH);
        break;
      case L2:
        elevator_.setEndEffectorHeight(
            FieldConstants.ReefHeight.L2.HEIGHT, FieldConstants.ReefHeight.L2.PITCH);
        break;
      case L3:
        elevator_.setEndEffectorHeight(
            FieldConstants.ReefHeight.L3.HEIGHT, FieldConstants.ReefHeight.L3.PITCH);
        break;
      case L4:
        elevator_.setEndEffectorHeight(
            FieldConstants.ReefHeight.L4.HEIGHT, FieldConstants.ReefHeight.L4.PITCH);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
