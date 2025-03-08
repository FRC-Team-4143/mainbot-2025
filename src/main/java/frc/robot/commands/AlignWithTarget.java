// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.mw_lib.command.DualConditionalCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithTarget extends ConditionalCommand {
  /** Creates a new ElevatorL4Target. */
  public AlignWithTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new CoralReefScore(),
        new ConditionalCommand(
            new DualConditionalCommand(
                new ScoreBarge(),
                new ScoreProcessor(),
                PoseEstimator.getInstance()::isBargeZone,
                PoseEstimator.getInstance()::isProcessorZone),
            new AlgaeReefPickup(),
            Claw.getInstance()::hasAlgae),
        Claw.getInstance()::isCoralMode);
  }
}
