package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.OI;
import frc.robot.OI.IntakePreference;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pickup.Pickup;
import frc.robot.subsystems.pose_estimator.PoseEstimator;

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
    // If in Climb Mode
    if (Climber.getInstance().getMode() != ClimberMode.DISABLED) {
      Elevator.getInstance().setTarget(TargetType.CLIMB);
      // If In Algae Mode
    } else if (Claw.getInstance().isAlgaeMode()) {
      Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
      // If In Coral Mode + Vision is Enabled
    } else if (OI.use_vision.getAsBoolean()) {
      if (Elevator.getInstance().getTarget() == TargetType.L4
          && PoseEstimator.getInstance().isInL4CollisionZone()) {
        // DO NOTHING
      } else {
        // If in Coral Mode + Vision is Enabled + Robot in Station Zone + Pickup Preference is
        // Station
        if (PoseEstimator.getInstance().isStationZone()
            && OI.intake_preference == IntakePreference.STATION) {
          Elevator.getInstance().setTarget(TargetType.STATION);
          // Waiting on Intake to get to position
        } else if (!Pickup.getInstance().isAtTarget()
            && OI.intake_preference == IntakePreference.GROUND) {
          Elevator.getInstance().setTarget(TargetType.SAFETY);
          // If in Coral Mode + Vision is Enabled + Robot Does Not Have Coral + Pickup Preference is
          // Ground
        } else if (!Claw.getInstance().isCoralPresent()
            && OI.intake_preference == IntakePreference.GROUND) {
          Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);

        } else {
          Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
        }
      }

      // If In Coral Mode + Vision is Disabled
    } else {
      // If in Coral Mode + Vision is Disabled + Robot Has Coral + Pickup Preference is Ground
      if (OI.intake_preference == IntakePreference.GROUND && Claw.getInstance().isCoralPresent()) {
        Elevator.getInstance().setTarget(TargetType.SAFETY);
        // If in Coral Mode + Vision is Disabled + Robot Does Not Have Coral + Pickup Preference is
        // Ground
      } else if (OI.intake_preference == IntakePreference.GROUND) {
        Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
        // If in Coral Mode + Vision is Disabled + Pickup Preference is Station
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
