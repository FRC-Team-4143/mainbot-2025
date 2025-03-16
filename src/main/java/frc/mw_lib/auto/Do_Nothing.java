package frc.mw_lib.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Do_Nothing extends Auto {

  public Do_Nothing() {
    this.addCommands(new WaitCommand(15));
  }
}
