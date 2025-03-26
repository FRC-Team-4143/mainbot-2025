package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.OI;
import frc.robot.OI.IntakePreference;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberMode;
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
    if (Climber.getInstance().getMode() != ClimberMode.DISABLED) {
      Elevator.getInstance().setTarget(TargetType.CLIMB);
    } else if (Claw.getInstance().isAlgaeMode()) {
      Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
    } else if (OI.use_vision.getAsBoolean()) {
      if (PoseEstimator.getInstance().isStationZone()
          && OI.intake_preference == IntakePreference.STATION) {
        Elevator.getInstance().setTarget(TargetType.STATION);
      } else if (!Claw.getInstance().isCoralPresent()
          && OI.intake_preference == IntakePreference.GROUND) {
        Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
      } else {
        Elevator.getInstance().setTarget(TargetType.CORAL_STOW);
      }
    } else {
      if (OI.intake_preference == IntakePreference.GROUND && Claw.getInstance().isCoralPresent()) {
        Elevator.getInstance().setTarget(TargetType.CORAL_STOW);
      } else if (OI.intake_preference == IntakePreference.GROUND) {
        Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
      } else {
        Elevator.getInstance().setTarget(TargetType.STATION);
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
