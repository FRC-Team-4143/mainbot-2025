// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

public class Clamp extends Command {

  public Clamp() {
    // Add requirements for command schedule interuption handling
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setClawMode(ClawMode.CLOSED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Claw.getInstance().setClawMode(ClawMode.OPEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // only stop if driver stops it
    return false;
    // Claw.getInstance().getClampAngle() > Constants.ClawConstants.OPEN_CLAW_ANGLE
    // || Claw.getInstance().getClampAmps() > averageAmps &&
    // Claw.getInstance().getClampAngle() >
    // Constants.ClawConstants.CLOSED_CLAW_ANGLE ;
  }
}
