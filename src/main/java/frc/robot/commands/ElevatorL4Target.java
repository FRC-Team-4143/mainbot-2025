// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.command.NoReqConditionalCommand;
import frc.robot.OI;
import frc.robot.commands.ManualElevatorOverride.Level;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.gsm.GameStateManager;
import frc.robot.subsystems.gsm.GameStateManager.ReefScoringTarget;
import frc.robot.subsystems.gsm.GameStateManager.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorL4Target extends NoReqConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public ElevatorL4Target() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        Commands.runOnce(
                () -> {
                  GameStateManager.getInstance().setScoringTarget(ReefScoringTarget.L4, true);
                  if (GameStateManager.getInstance().isRunning()
                      && Claw.getInstance().isCoralMode()) {
                    GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
                  }
                })
            .ignoringDisable(true),
        new ManualElevatorOverride(Level.L4),
        OI.use_vision);
    setName(this.getClass().getSimpleName());
  }
}
