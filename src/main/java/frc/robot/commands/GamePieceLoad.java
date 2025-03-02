// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.OI;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GamePieceLoad extends ConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public GamePieceLoad() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new ConditionalCommand(new CoralStation(), new CoralLoad(), OI.use_vision),
        new AlgaeLoad(),
        Claw.getInstance()::isCoralMode);
  }
}
