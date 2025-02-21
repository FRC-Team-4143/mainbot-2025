// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.SwerveModuleConstants;
import frc.mw_lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.mw_lib.swerve.SwerveModuleConstantsFactory;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public class DrivetrainConstants {

    // Can bus names for each of the swerve modules
    public static final String[] MODULE_CANBUS_NAME = {"rio", "rio", "rio", "rio"};

    // Can bus ID for the pigeon
    public static final int PIGEON2_ID = 0;

    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control

    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs().withKP(120).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0.0).withKS(0.0).withKV(0.12).withKA(0.00);

    private static final double SLIP_CURRENT_AMPS = 50;
    private static final double SPEED_AT_12V_MPS = 4.4;
    private static final double COUPLE_RATIO = 3.5;
    private static final double DRIVE_GEAR_RATIO = 6.12;
    private static final double STEER_GEAR_RATIO = 21.4285714286;
    private static final double WHEEL_RADIUS_INCH = 1.8;
    private static final boolean STEER_MOTOR_REVERSED = true;
    public static final double MAX_DRIVE_SPEED = 4.4;
    public static final double MAX_DRIVE_ANGULAR_RATE = 6.28;

    public static final double CRAWL_DRIVE_SPEED = MAX_DRIVE_SPEED * 0.1;
    public static final double MAX_TARGET_SPEED = 1;

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS_INCH)
            .withSlipCurrent(SLIP_CURRENT_AMPS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSpeedAt12VoltsMps(SPEED_AT_12V_MPS)
            .withFeedbackSource(
                SteerFeedbackType.None) // .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            // CRH: Removed
            // for AnalogEncoders
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage);

    public static final SwerveModuleConstants FL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            2, 1, 0, 0, Units.inchesToMeters(7.5), Units.inchesToMeters(7.5), false);
    public static final SwerveModuleConstants FR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(4, 3, 1, 0, 7.5, -7.5, false);
    public static final SwerveModuleConstants BL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(6, 5, 2, 0, -7.5, 7.5, false);
    public static final SwerveModuleConstants BR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(7, 8, 3, 0, -7.5, -7.5, false);

    // Drivetrain PID Controller
    public static final PIDController X_TRAJECTORY_TRANSLATION = new PIDController(0.5, 0.0, 0.0);
    public static final PIDController Y_TRAJECTORY_TRANSLATION = new PIDController(0.5, 0.0, 0.0);
    public static final PIDController TRAJECTORY_HEADING = new PIDController(2.0, 0.0, 0.0);
    public static final PIDController X_POSE_TRANSLATION = new PIDController(1.5, 0.0, 0.0);
    public static final PIDController Y_POSE_TRANSLATION = new PIDController(1.5, 0.0, 0.0);
    public static final PIDController POSE_HEADING = new PIDController(2.0, 0.0, 0.0);

    public static final double CENTER_OFFSET_X = 13.5;
  }

  public static final class FeederConstants {
    public static final int LEFT_FEEDER_MOTOR = 11;
    public static final int RIGHT_FEEDER_MOTOR = 10;
    public static final boolean LEFT_FEEDER_INVERTED = true;
    public static final boolean RIGHT_FEEDER_INVERTED = false;
    public static final double FEEDER_SPEED = 0.15;
    public static final double SCORE_SPEED = 0.4;
    public static final double IDLE_SPEED = 0;
    public static final double AMP_SPIKE_THRESHHOLD = 25;
  }
}
