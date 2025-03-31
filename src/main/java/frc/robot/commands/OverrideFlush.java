// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OverrideFlush extends Command {
  static Pickup pickup_;

  public OverrideFlush() {
    // Use addRequirements() here to declare subsystem dependencies.
    pickup_ = Pickup.getInstance();
    addRequirements(pickup_);
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup_.setPickupMode(PickupMode.FLUSH_OUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pickup_.setPickupMode(PickupMode.DEPLOYED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
