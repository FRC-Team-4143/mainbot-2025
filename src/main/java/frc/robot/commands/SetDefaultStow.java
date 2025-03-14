package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.Target;
import frc.robot.OI;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;

public class SetDefaultStow extends Command {
  public SetDefaultStow() {
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Claw.getInstance().isAlgaeMode()) {
      Elevator.getInstance().setTarget(Target.ALGAE_STOW);
    } else {
      if (OI.use_vision.getAsBoolean()) {
        if (PoseEstimator.getInstance().isStationZone()) {
          Elevator.getInstance().setTarget(Target.STOW);
        } else {
          Elevator.getInstance().setTarget(Target.CORAL_STOW);
        }
      } else {
        Elevator.getInstance().setTarget(Target.STOW);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
