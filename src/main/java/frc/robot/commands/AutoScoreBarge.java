// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.ScoringPoses;
import frc.mw_lib.geometry.Region;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawMode;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.drive.SwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveDrivetrain.DriveMode;
import java.util.Optional;

public class AutoScoreBarge extends Command {

  static Optional<Region> current_region = Optional.empty();

  /** Creates a new CoralStationLoad. */
  public AutoScoreBarge() {
    addRequirements(Elevator.getInstance(), SwerveDrivetrain.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
    SwerveDrivetrain.getInstance().setTightRope(ScoringPoses.BARGE_TIGHT_ROPE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Elevator.getInstance().isElevatorAndArmAtTarget()) {
      Claw.getInstance().setClawMode(ClawMode.SHOOT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Claw.getInstance().hasAlgae();
  }
}
