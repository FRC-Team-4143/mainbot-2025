// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.command.NoReqConditionalCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralScore extends NoReqConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public CoralScore() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new L1Score(),
        new CoralEject(),
        Elevator.getInstance()::targetIsL1);
    setName(this.getClass().getSimpleName());
  }
}
