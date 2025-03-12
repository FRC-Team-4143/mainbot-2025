package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.OI;
import frc.robot.commands.ManualElevatorOverride.Level;
import frc.robot.subsystems.Elevator;


public class SetDefaultPose extends Command {
  public SetDefaultPose() {
    Elevator.getInstance().stowElevator();
    setName(this.getClass().getSimpleName());
  }
}
