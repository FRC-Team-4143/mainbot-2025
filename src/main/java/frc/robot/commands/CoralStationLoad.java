// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Elevator;

public class CoralStationLoad extends Command {

  static Claw claw_;
  static Elevator elevator_;

  /** Creates a new CoralStationLoad. */
  public CoralStationLoad() {
    claw_ = Claw.getInstance();
    elevator_ = Elevator.getInstance();
    addRequirements(claw_, elevator_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw_.setClawMode(ClawMode.LOAD);
    elevator_.setEndEffectorHeight(Units.inchesToMeters(28), Rotation2d.fromDegrees(-40));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_.stowElevator();
    claw_.setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
