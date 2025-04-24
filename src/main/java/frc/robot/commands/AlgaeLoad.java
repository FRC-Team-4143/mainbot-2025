// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawMode;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.pickup.Pickup;
import frc.robot.subsystems.pickup.Pickup.PickupMode;

public class AlgaeLoad extends Command {

  public AlgaeLoad() {
    addRequirements(Claw.getInstance());
    setName(this.getClass().getSimpleName());
  }

  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
    Claw.getInstance().setClawMode(ClawMode.LOAD);
  }

  @Override
  public void execute() {
    if (!Pickup.getInstance().isCoralPresent()) {
      Pickup.getInstance().setPickupMode(PickupMode.INTAKE);
    } else {
      Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
