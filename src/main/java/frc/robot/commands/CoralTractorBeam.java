// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PickupConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupMode;
import frc.robot.subsystems.SwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralTractorBeam extends Command {
  /** Creates a new CoralTractorBeam. */
  static Transform2d intake_off_set;

  static Pose2d target_;
  static Transform2d stageing_off_set_;
  static boolean has_hit_staging_target_;

  public CoralTractorBeam() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake_off_set =
        new Transform2d(
            DrivetrainConstants.CENTER_OFFSET_X,
            -PickupConstants.INTAKE_OFF_SET_Y,
            Rotation2d.kZero);
    stageing_off_set_ =
        new Transform2d(
            DrivetrainConstants.CENTER_OFFSET_X + Units.inchesToMeters(12),
            -PickupConstants.INTAKE_OFF_SET_Y,
            Rotation2d.kZero);
    addRequirements(Pickup.getInstance(), Claw.getInstance(), Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ProxyServer.snapshot("Coral Tractor Beam Enabled");
    Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
    Pickup.getInstance().setPickupMode(PickupMode.INTAKE);
    Claw.getInstance().setGamePiece(GamePiece.CORAL);
    target_ = CoralDetector.getInstance().getCoralPose2d().transformBy(intake_off_set);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Elevator.getInstance().isElevatorAndArmAtTarget() == true) {
      Pickup.getInstance().setPickupMode(PickupMode.INTAKE);
      Claw.getInstance().setClawMode(ClawMode.LOAD);
      if (CoralDetector.getInstance().isValid()) {
        target_ = CoralDetector.getInstance().getCoralPose2d().transformBy(intake_off_set);
        if (!Pickup.getInstance().isCoralPresent()) {
          SwerveDrivetrain.getInstance().setTargetFollow(target_);
        } else {
          SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
        }
      }
    }
    if (Pickup.getInstance().isCoralPresent()) {
      SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    Pickup.getInstance().setPickupMode(PickupMode.DEPLOYED);
    Claw.getInstance().setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Pickup.getInstance().isCoralPresent() || Claw.getInstance().isCoralPresent();
  }
}
