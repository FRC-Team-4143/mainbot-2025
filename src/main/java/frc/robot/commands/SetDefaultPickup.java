package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.OI.IntakePreference;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberMode;
import frc.robot.subsystems.pickup.Pickup;
import frc.robot.subsystems.pickup.Pickup.PickupMode;

public class SetDefaultPickup extends Command {
  public SetDefaultPickup() {
    addRequirements(Pickup.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Climber.getInstance().getMode() == ClimberMode.DISABLED) {
      if (Claw.getInstance().getGamePieceMode() == GamePiece.ALGAE) {
        Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
      } else {
        if (OI.intake_preference == IntakePreference.GROUND) {
          Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
        } else {
          Pickup.getInstance().setPickupMode(PickupMode.STATION);
        }
      }
    } else {
      // Climb
      // DO NOTHING (Let the Climber State Machine Handle Control)
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
