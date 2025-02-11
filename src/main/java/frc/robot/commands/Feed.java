package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.CoralFunnel.FeedingMode;

public class Feed extends Command {
  private int counter = 0;

  public Feed() {
    // Add requirements for command schedule interuption handling
    // addRequirements(subsystem);
    addRequirements(CoralFunnel.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CoralFunnel.getInstance().setFeedingMode(FeedingMode.FEEDING);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralFunnel.getInstance().setFeedingMode(FeedingMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralFunnel.getInstance().hasCoral() && (counter >= 25);
  }
}
