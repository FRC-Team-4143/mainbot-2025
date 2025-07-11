// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.geometry.Region;
import frc.mw_lib.util.NumUtil;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.Optional;

public class AutoScoreBarge extends Command {

  static Optional<Region> current_region = Optional.empty();
  static int cyclesToWaitAfter = 15;
  static int counter;
  static boolean amDone;

  /** Creates a new CoralStationLoad. */
  public AutoScoreBarge() {
    addRequirements(Elevator.getInstance(), SwerveDrivetrain.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
    Elevator.getInstance().setTarget(TargetType.BARGE);
    SwerveDrivetrain.getInstance()
        .setTargetRotation(SwerveDrivetrain.getInstance().getDriverPerspective());

    counter = 0;
    amDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Elevator.getInstance().isElevatorAndArmAtTarget()
        && NumUtil.epislonEquals(
            PoseEstimator.getInstance().getRobotPose().getRotation(),
            SwerveDrivetrain.getInstance().getDriverPerspective(),
            Units.degreesToRadians(2))) {
      amDone = true;
      Claw.getInstance().setClawMode(ClawMode.BLAST);
    }
    if (amDone) {
      counter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return amDone && (counter >= cyclesToWaitAfter);
  }
}
