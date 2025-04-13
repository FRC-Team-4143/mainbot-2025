// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.command.NoReqSequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1Score extends NoReqSequentialCommandGroup {
  /** Creates a new ElevatorL4Target. */
  public L1Score() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.L1)),
        Commands.runOnce(() -> Claw.getInstance().setClawMode(ClawMode.SHOOT)),
        new WaitCommand(0.25),
        Commands.runOnce(() -> Claw.getInstance().setClawMode(ClawMode.IDLE)));
    addRequirements(Elevator.getInstance());
    addRequirements(Claw.getInstance());
    setName(this.getClass().getSimpleName());
  }
}
