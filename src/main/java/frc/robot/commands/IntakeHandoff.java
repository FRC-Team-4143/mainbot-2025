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

  /** Creates a new CoralEject. */
  public IntakeHandoff() {
    addRequirements(Pickup.getInstance());
    addRequirements(Claw.getInstance());
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.CORAL);
    Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
    Pickup.getInstance().setPickupMode(PickupMode.INTAKE);
    Claw.getInstance().setClawMode(ClawMode.LOAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
    Claw.getInstance().setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Claw.getInstance().hasCoral()
        || (Pickup.getInstance().isCoralPresent()
            && !(Elevator.getInstance().getTarget() == TargetType.CORAL_INTAKE
                && Elevator.getInstance().isElevatorAndArmAtTarget())));
  }
}
