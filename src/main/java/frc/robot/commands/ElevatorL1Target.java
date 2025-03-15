// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.OI;
import frc.robot.commands.ManualElevatorOverride.Level;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorL1Target extends ConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public ElevatorL1Target() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        Commands.runOnce(
            () -> GameStateManager.getInstance().setScoringTarget(ReefScoringTarget.L1, true)),
        new ManualElevatorOverride(Level.L1),
        OI.use_vision);
    setName(this.getClass().getSimpleName());
  }
}
