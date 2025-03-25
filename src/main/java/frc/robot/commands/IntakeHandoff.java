// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupMode;

public class IntakeHandoff extends Command {

  static Pickup pickup_;
  static Claw claw_;
  static Elevator elevator_;

  /** Creates a new CoralEject. */
  public IntakeHandoff() {
    pickup_ = Pickup.getInstance();
    elevator_ = Elevator.getInstance();
    claw_ = Claw.getInstance();
    addRequirements(pickup_);
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_.setTarget(TargetType.CORAL_INTAKE);
    pickup_.setPickupMode(PickupMode.DEPLOYED);
    claw_.setGamePiece(GamePiece.CORAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator_.isElevatorAndArmAtTarget() == true) {
      pickup_.setPickupMode(PickupMode.INTAKE);
      claw_.setClawMode(ClawMode.LOAD);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pickup_.setPickupMode(PickupMode.DEPLOYED);
    claw_.setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw_.hasCoral();
  }
}
