// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.mw_lib.command.NoReqConditionalCommand;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GamePieceEject extends NoReqConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public GamePieceEject() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new CoralScore(), new AlgaeEject(), Claw.getInstance()::isCoralMode);
    setName(this.getClass().getSimpleName());
  }
}
