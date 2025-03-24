// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OverrideLoad extends Command {
  static Claw claw_;

  /**
   * Creates a new OverrideLoad.
   *
   * @param set_to_idle_ sets the mode back to idle on true or whatever clawmode it was at the
   *     beginning of the command on false
   */
  public OverrideLoad() {
    // Use addRequirements() here to declare subsystem dependencies.
    claw_ = Claw.getInstance();
    addRequirements(claw_);
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw_.setClawMode(ClawMode.LOAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw_.setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
