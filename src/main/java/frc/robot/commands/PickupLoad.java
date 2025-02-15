// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.mw_lib.command.LazyCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupStages;

public class PickupLoad extends Command {
     
    static Pickup pickup_;
    
  public PickupLoad() {
    // Add requirements for command schedule interuption handling
    //addRequirements(subsystem);
    pickup_ = Pickup.getInstance();
    addRequirements(pickup_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup_.setPickupStage(pickup_.setPickupStage(Pickup.PICKUP));
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   Pickup.setIdlePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}