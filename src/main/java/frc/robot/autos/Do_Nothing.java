package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.mw_lib.auto.Auto;

public class Do_Nothing extends Auto {

  public Do_Nothing() {
    this.addCommands(new WaitCommand(15));
  }
}
