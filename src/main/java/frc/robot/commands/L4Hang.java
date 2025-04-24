// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4Hang extends Command {
  private TargetType safety = TargetType.L4_SAFETY;

  /** Creates a new L4Hang. */
  public L4Hang() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    safety.arm_angle_ = Rotation2d.fromRadians(Elevator.getInstance().getCurrentAngle());
    Elevator.getInstance().setTarget(safety);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().setTarget(TargetType.L4);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Elevator.getInstance().isElevatorAndArmAtTarget();
  }
}
