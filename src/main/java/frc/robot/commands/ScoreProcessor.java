// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.FieldRegions;
import frc.mw_lib.geometry.Region;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.Optional;

public class ScoreProcessor extends Command {

  static Optional<Region> current_region = Optional.empty();

  /** Creates a new CoralStationLoad. */
  public ScoreProcessor() {
    addRequirements(Elevator.getInstance(), SwerveDrivetrain.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
    Elevator.getInstance().setSpeedLimit(SpeedLimit.ALGAE);
    // Claw.getInstance().enableBlastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (FieldRegions.PROCESSOR_REGION.contains(PoseEstimator.getInstance().getRobotPose())) {
      Elevator.getInstance().setTarget(TargetType.ALGAE_PROCESSOR);
    } else {
      Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
    Claw.getInstance().disableBlastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
