package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupMode;

public class SetIntake extends Command {

  public SetIntake() {
    addRequirements(Pickup.getInstance());
  }

  @Override
  public void initialize() {
    Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Pickup.getInstance().setPickupMode(PickupMode.STATION);
  }
}
