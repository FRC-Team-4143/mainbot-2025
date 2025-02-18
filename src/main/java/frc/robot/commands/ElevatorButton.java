// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Target;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorButton extends Command {

  public enum Level {
    L1,
    L2,
    L3,
    L4
  }

  private Level level_;

  /** Creates a new LowButtonCommand. */
  public ElevatorButton(Level level) {
    level_ = level;
    addRequirements(Elevator.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Claw.getInstance().isCoralMode()) {
      Elevator.getInstance().setSpeedLimit(SpeedLimit.CORAL);

    } else {
      Elevator.getInstance().setSpeedLimit(SpeedLimit.ALGAE);
      Elevator.getInstance().setTarget(Target.ALGAE_PROCESSOR);
    }

    switch (level_) {
      case L1:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? Target.STATION : Target.ALGAE_PROCESSOR);
        break;

      case L2:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? Target.L2 : Target.ALGAE_LOW);
        break;

      case L3:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? Target.L3 : Target.ALGAE_HIGH);
        break;

      case L4:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? Target.L4 : Target.BARGE);
        break;
      default:
        // hello :)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Claw.getInstance().isCoralMode()) {
      Elevator.getInstance().setTarget(Target.STOW);
    } else {
      Elevator.getInstance().setTarget(Target.ALGAE_PROCESSOR);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return level_ == Level.L4
        && Claw.getInstance().isAlgaeMode()
        && !Elevator.getInstance().canExtendForBarge();
  }
}
