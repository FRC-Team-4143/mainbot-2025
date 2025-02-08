// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.mw_lib.swerve.SwerveModule;
import frc.mw_lib.swerve.SwerveRequest;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

public class SwerveProfile extends Command {

  double x_speed_req = 0.0;
  double y_speed_req = 0.0;
  double rot_speed_req = 0.0;

  /** Creates a new SwerveProfile. */
  public SwerveProfile(double x_speed, double y_speed, double rot_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    x_speed_req = x_speed;
    y_speed_req = y_speed;
    rot_speed_req = rot_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.PROFILE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveDrivetrain.getInstance()
        .setControl(
            new SwerveRequest.ApplyChassisSpeeds()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSpeeds(
                    new ChassisSpeeds(
                        x_speed_req,
                        y_speed_req,
                        rot_speed_req))); // 3 m/s in x direction (frame relative)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
