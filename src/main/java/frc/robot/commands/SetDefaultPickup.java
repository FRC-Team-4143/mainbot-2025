package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.OI.IntakePreference;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberMode;
import frc.robot.subsystems.Pickup.PickupMode;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Claw.GamePiece;

public class SetDefaultPickup extends Command {
  public SetDefaultPickup() {
    addRequirements(Pickup.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

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
      Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
