// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.RobotState;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralReefScore extends SequentialCommandGroup {
  /** Creates a new AutoScoreCoral. */
  public AutoCoralReefScore() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new GetToReefTarget(),
        new CoralEject().withTimeout(0.5).beforeStarting(new WaitCommand(0.2)),
        Commands.runOnce(() -> GameStateManager.getInstance().setRobotState(RobotState.END)));

    setName(this.getClass().getSimpleName());
  }

  public class GetToReefTarget extends Command {

    public GetToReefTarget() {

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(Elevator.getInstance());
      addRequirements(Claw.getInstance());
      addRequirements(SwerveDrivetrain.getInstance());

      setName(this.getClass().getSimpleName());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return GameStateManager.getInstance().getRobotState() == RobotState.SCORING;
    }
  }
}
