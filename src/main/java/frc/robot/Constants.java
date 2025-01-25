// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.SwerveModuleConstants;
import frc.mw_lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.mw_lib.swerve.SwerveModuleConstantsFactory;
import frc.mw_lib.util.ConstantsLoader;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

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
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs().withKP(100).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(6.25)
            .withKI(0.0)
            .withKD(0.01) // 7 : updated to 3 RJS
            .withKS(0.2)
            .withKV(0.12)
            .withKA(0.05); // 2.4 : updated to 0 RJS

    private static final double SLIP_CURRENT_AMPS =
        LOADER.getDoubleValue("drive", "com", "SLIP_CURRENT");
    private static final double SPEED_AT_12V_MPS =
        LOADER.getDoubleValue("drive", "com", "SPEED_AT_12V");
    private static final double COUPLE_RATIO =
        LOADER.getDoubleValue("drive", "com", "COUPLE_RATIO");
    private static final double DRIVE_GEAR_RATIO =
        LOADER.getDoubleValue("drive", "com", "DRIVE_GEAR_RATIO");
    private static final double STEER_GEAR_RATIO =
        LOADER.getDoubleValue("drive", "com", "STEER_GEAR_RATIO");
    private static final double WHEEL_RADIUS_INCH =
        LOADER.getDoubleValue("drive", "com", "WHEEL_RADIUS_INCH");
    private static final boolean STEER_MOTOR_REVERSED =
        LOADER.getBoolValue("drive", "com", "STEER_MOTOR_REVERSED");
    public static final double MAX_DRIVE_SPEED =
        LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_SPEED");
    public static final double MAX_DRIVE_ANGULAR_RATE =
        LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_ANGULAR_RATE");
    public static final double CRAWL_DRIVE_SPEED = 0.4;
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
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC);

    public static final SwerveModuleConstants FL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "fl", "STEER_ID"),
            LOADER.getIntValue("drive", "fl", "DRIVE_ID"),
            LOADER.getIntValue("drive", "fl", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "Y_POSITION")),
            LOADER.getBoolValue("drive", "fl", "INVERT_DRIVE"));
    public static final SwerveModuleConstants FR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "fr", "STEER_ID"),
            LOADER.getIntValue("drive", "fr", "DRIVE_ID"),
            LOADER.getIntValue("drive", "fr", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "Y_POSITION")),
            LOADER.getBoolValue("drive", "fr", "INVERT_DRIVE"));
    public static final SwerveModuleConstants BL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "bl", "STEER_ID"),
            LOADER.getIntValue("drive", "bl", "DRIVE_ID"),
            LOADER.getIntValue("drive", "bl", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "Y_POSITION")),
            LOADER.getBoolValue("drive", "bl", "INVERT_DRIVE"));
    public static final SwerveModuleConstants BR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "br", "STEER_ID"),
            LOADER.getIntValue("drive", "br", "DRIVE_ID"),
            LOADER.getIntValue("drive", "br", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "Y_POSITION")),
            LOADER.getBoolValue("drive", "br", "INVERT_DRIVE"));
  }

  public static final class FeederConstants {
    public static final int LEFT_FEEDER_MOTOR = 11;
    public static final int RIGHT_FEEDER_MOTOR = 10;
    public static final boolean LEFT_FEEDER_INVERTED = true;
    public static final boolean RIGHT_FEEDER_INVERTED = false;
    public static final double FEEDER_SPEED = 0.15;
    public static final double SCORE_SPEED = 0.5;
    public static final double IDLE_SPEED = 0;
    public static final double AMP_SPIKE_THRESHHOLD = 25;
  }

  public class ElevatorConstants {
    // Elevator Constants
    public static final int ELEVATOR_MASTER_ID = 21;
    public static final int ELEVATOR_FOLLOWER_ID = 22;
    public static final int ELEVATOR_LIMIT_SWITCH_PORT_NUMBER = 4;
    public static final double ELEVATOR_TARGET_THRESHOLD = 0.25; // In m
    public static final double ELEVATOR_MAX_HEIGHT = 0.0; // In m
    public static final InvertedValue ELEVATOR_MASTER_INVERSION_ = InvertedValue.Clockwise_Positive;
    public static final InvertedValue ELEVATOR_FOLLOWER_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 0;
    public static final double ELEVATOR_CRUISE_VELOCITY = 0;
    public static final double ELEVATOR_ACCELERATION = 0;
    public static final double ELEVATOR_EXPO_KV = 0;
    public static final double ELEVATOR_EXPO_KA = 0;
    public static final double ELEVATOR_ZERO_THRESHOLD = 0; // In m
    public static final double ROTOR_TO_CENSOR_RATIO = 1;

    // Arm Constants:
    public static final int ARM_MOTOR_ID = 23;
    public static final int ARM_ENCODER_ID = 24;
    public static final double ARM_TARGET_THRESHOLD = 0.25; // In rads
    public static final InvertedValue ARM_FOLLOWER_INVERSION = InvertedValue.Clockwise_Positive;
    public static final double ARM_HOME_POSITION = 0;
    public static final double ARM_SENSOR_TO_MECHANISM_RATIO = 0;
    public static final double ARM_CRUISE_VELOCITY = 0;
    public static final double ARM_ACCELERATION = 0;
    public static final double ARM_EXPO_KV = 0;
    public static final double ARM_EXPO_KA = 0;
    public static final double ARM_LOWER_LIMIT = 0;

    public static final Slot0Configs ELEVATOR_GAINS =
        new Slot0Configs()
            .withKP(0.0)
            .withKI(0.0) // <-DO NOT TOUCH!!!!!!!!!
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final Slot0Configs ARM_GAINS =
        new Slot0Configs()
            .withKP(0.0)
            .withKI(0.0) // DO NOT TOUCH!!!!!!!!!
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);
  }
}
