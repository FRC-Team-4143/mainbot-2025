// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.SwerveModuleConstants;
import frc.mw_lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.mw_lib.swerve.SwerveModuleConstantsFactory;
import frc.mw_lib.swerve.utility.ModuleType;
import frc.mw_lib.util.CamConstants;
import frc.mw_lib.util.ConstantsLoader;
import frc.mw_lib.util.TagLayouts;
import java.util.List;

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

  public static class Vision {
    public static final List<CamConstants> CAMERAS = LOADER.getCameras("vision");

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        TagLayouts.getTagLayoutFromPath("apriltagLayouts/onlyReef.json");

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2.0, 2.0, 4);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  }

  public class DrivetrainConstants {
    // Can bus names for each of the swerve modules
    public static final String[] MODULE_CANBUS_NAME = {
      "CANivore", "CANivore", "CANivore", "CANivore"
    };

    // Can bus ID for the pigeon
    public static final int PIGEON2_ID = 0;

    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control

    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_P"))
            .withKI(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_I"))
            .withKD(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_D"))
            .withKS(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_S"))
            .withKV(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_V"))
            .withKA(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_A"));
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_P"))
            .withKI(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_I"))
            .withKD(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_D"))
            .withKS(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_S"))
            .withKV(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_V"))
            .withKA(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_A"));

    private static final double SLIP_CURRENT_AMPS =
        LOADER.getDoubleValue("drive", "com", "SLIP_CURRENT");
    private static final double SPEED_AT_12V_MPS =
        LOADER.getDoubleValue("drive", "com", "SPEED_AT_12V");
    private static final double COUPLE_RATIO =
        LOADER.getDoubleValue("drive", "com", "COUPLE_RATIO");
    private static final double WHEEL_RADIUS_INCH =
        LOADER.getDoubleValue("drive", "com", "WHEEL_RADIUS_INCH");
    private static final boolean STEER_MOTOR_REVERSED =
        LOADER.getBoolValue("drive", "com", "STEER_MOTOR_REVERSED");
    public static final double MAX_DRIVE_SPEED =
        LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_SPEED");
    public static final double MAX_DRIVE_ANGULAR_RATE =
        LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_ANGULAR_RATE");

    public static final double CRAWL_DRIVE_SPEED = MAX_DRIVE_SPEED * 0.1;
    public static final double MAX_TARGET_SPEED = 1;
    public static final double MAX_TRACTOR_BEAM_VELOCITY_SPEED = MAX_DRIVE_SPEED * 0.35;
    public static final double MAX_TRACTOR_BEAM_OMEGA_SPEED = MAX_DRIVE_ANGULAR_RATE * 0.6;
    public static final double TRACTOR_BEAM_ROTATION_THRESHOLD = Units.degreesToRadians(2);
    public static final double TRACTOR_BEAM_TARGET_DISTANCE = Units.inchesToMeters(1);

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withWheelRadius(WHEEL_RADIUS_INCH)
            .withSlipCurrent(SLIP_CURRENT_AMPS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSpeedAt12VoltsMps(SPEED_AT_12V_MPS)
            .withFeedbackSource(SteerFeedbackType.None) // Analog Encoders
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage);

    public static final SwerveModuleConstants FL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "fl", "STEER_ID"),
            LOADER.getIntValue("drive", "fl", "DRIVE_ID"),
            LOADER.getIntValue("drive", "fl", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "Y_POSITION")),
            LOADER.getBoolValue("drive", "fl", "INVERT_DRIVE"),
            ModuleType.getModuleType("fl"));
    public static final SwerveModuleConstants FR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "fr", "STEER_ID"),
            LOADER.getIntValue("drive", "fr", "DRIVE_ID"),
            LOADER.getIntValue("drive", "fr", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "Y_POSITION")),
            LOADER.getBoolValue("drive", "fr", "INVERT_DRIVE"),
            ModuleType.getModuleType("fr"));
    public static final SwerveModuleConstants BL_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "bl", "STEER_ID"),
            LOADER.getIntValue("drive", "bl", "DRIVE_ID"),
            LOADER.getIntValue("drive", "bl", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "Y_POSITION")),
            LOADER.getBoolValue("drive", "bl", "INVERT_DRIVE"),
            ModuleType.getModuleType("bl"));
    public static final SwerveModuleConstants BR_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            LOADER.getIntValue("drive", "br", "STEER_ID"),
            LOADER.getIntValue("drive", "br", "DRIVE_ID"),
            LOADER.getIntValue("drive", "br", "ENCODER_ID"),
            0,
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "X_POSITION")),
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "Y_POSITION")),
            LOADER.getBoolValue("drive", "br", "INVERT_DRIVE"),
            ModuleType.getModuleType("br"));

    // Drivetrain PID Controller
    public static final PIDController X_TRAJECTORY_TRANSLATION =
        new PIDController(
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_P"),
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_I"),
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_D"));
    public static final PIDController Y_TRAJECTORY_TRANSLATION =
        new PIDController(
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_P"),
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_I"),
            LOADER.getDoubleValue("drive", "traj_controller", "TRANSLATION_D"));
    public static final PIDController TRAJECTORY_HEADING =
        new PIDController(
            LOADER.getDoubleValue("drive", "traj_controller", "HEADING_P"),
            LOADER.getDoubleValue("drive", "traj_controller", "HEADING_I"),
            LOADER.getDoubleValue("drive", "traj_controller", "HEADING_D"));
    public static final PIDController X_POSE_TRANSLATION =
        new PIDController(
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_P"),
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_I"),
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_D"));
    public static final PIDController Y_POSE_TRANSLATION =
        new PIDController(
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_P"),
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_I"),
            LOADER.getDoubleValue("drive", "pose_controller", "TRANSLATION_D"));
    public static final PIDController POSE_HEADING =
        new PIDController(
            LOADER.getDoubleValue("drive", "pose_controller", "HEADING_P"),
            LOADER.getDoubleValue("drive", "pose_controller", "HEADING_I"),
            LOADER.getDoubleValue("drive", "pose_controller", "HEADING_D"));

    public static final double CENTER_OFFSET_X =
        Units.inchesToMeters(LOADER.getDoubleValue("drive", "com", "CENTER_OFFSET_X"));
  }

  public static final class ClawConstants {
    public static final double TIME_OF_FLIGHT_DIST = 50;
    public static final int WHEEL_MOTOR_ID = 11;
    public static final double WHEEL_CORAL_SHOOT_SPEED = 0.3;
    public static final double WHEEL_CORAL_BLAST_SPEED = 0.6;
    public static final double WHEEL_ALGAE_SHOOT_SPEED = 0.5;
    public static final double WHEEL_ALGAE_BLAST_SPEED = 0.5;
    public static final double CORAL_LOAD_SPEED = -0.5;
    public static final double ALGAE_LOAD_SPEED = -0.3;
    public static final double ALGAE_IDLE_SPEED = 0.1;
    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final String CORAL_COLOR = new Color(255, 255, 255).toHexString();
    public static final String ALGAE_COLOR = new Color(0, 255, 255).toHexString();
    public static final InvertedValue WHEEL_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final double CORAL_IMP_OFFSET =
        Units.inchesToMeters(LOADER.getDoubleValue("imp", "coral_offset"));
    public static final double ALGAE_IMP_OFFSET =
        Units.inchesToMeters(LOADER.getDoubleValue("imp", "algae_offset"));
    public static final double CORAL_CURRENT_THRESHOLD = 4.25;
  }

  public class ClimberConstants {
    public static final int STRAP_ID = 31;
    public static final int PRONG_ID = 1;
    public static final int PRONG_ID_A = 8;
    public static final int PRONG_ID_B = 9;
    public static final int ARM_ID = 0;
    public static final InvertedValue STRAP_INVERSION = InvertedValue.Clockwise_Positive;
    public static final Slot0Configs STRAP_GAINS =
        new Slot0Configs().withKP(0.16).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0);
    public static final double PRONG_DEPLOY_SPEED = 0.4;
    public static final double PRONG_HOLD_SPEED = -0.4;
    public static final double ARM_DEPLOY_SPEED = 0.7;
    public static final double ARM_HOLD_SPEED = 0.5;
    public static final double STRAP_RETRACTED_POSITION = 85;
    public static final double STRAP_SETPOINT_BUMP = (STRAP_RETRACTED_POSITION / 25.0);
    public static final double PRONG_PRESET_COUNT = 17;
    public static final double DEPLOYING_TIME = 1.2;
    public static final double PRONG_P = 0.030;
    public static final double PRONG_D = 0.002;
  }

  public class ElevatorConstants {
    // Elevator Constants
    public static final int ELEVATOR_MASTER_ID = 21;
    public static final int ELEVATOR_FOLLOWER_ID = 22;
    public static final int ELEVATOR_LIMIT_SWITCH_PORT_NUMBER = 4;
    public static final double ELEVATOR_TARGET_THRESHOLD = Units.inchesToMeters(1); // In m
    public static final InvertedValue ELEVATOR_MASTER_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ELEVATOR_FOLLOWER_INVERSION =
        InvertedValue.Clockwise_Positive;
    // Units.inchesToMeters(Sprocket Circumference * Math.PI) / gearbox ratio *
    // rigging
    public static final double ELEVATOR_ROTATIONS_TO_METERS =
        Units.inchesToMeters(1.751 * Math.PI)
            / LOADER.getDoubleValue("elevator", "ELEVATOR_GEAR_RATIO")
            * 2;
    public static final double ELEVATOR_CRUISE_VELOCITY = 5.0 / ELEVATOR_ROTATIONS_TO_METERS;
    public static final double ELEVATOR_ACCEL = 3.0 / ELEVATOR_ROTATIONS_TO_METERS;
    public static final double ELEVATOR_EXPO_KV = 0.11733;
    public static final double ELEVATOR_EXPO_KA = 0.0070285;
    public static final double ELEVATOR_ZERO_THRESHOLD = 0; // In m
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 80.0;
    public static final double ELEVATOR_HEIGHT_PIVOT_MIN =
        Units.inchesToMeters(LOADER.getDoubleValue("elevator", "HEIGHT_PIVOT_MIN"));
    public static final double ELEVATOR_HEIGHT_PIVOT_MAX =
        Units.inchesToMeters(LOADER.getDoubleValue("elevator", "HEIGHT_PIVOT_MAX"))
            - 0.1; // 0.1m of safety
    public static final double ELEVATOR_HEIGHT_PIVOT_SAFETY =
        ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(6);

    public static final Slot0Configs ELEVATOR_GAINS =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("elevator", "CONTROLLER_P"))
            .withKI(LOADER.getDoubleValue("elevator", "CONTROLLER_I"))
            .withKD(LOADER.getDoubleValue("elevator", "CONTROLLER_D"))
            .withKS(LOADER.getDoubleValue("elevator", "CONTROLLER_S"))
            .withKV(LOADER.getDoubleValue("elevator", "CONTROLLER_V"))
            .withKA(LOADER.getDoubleValue("elevator", "CONTROLLER_A"))
            .withKG(LOADER.getDoubleValue("elevator", "CONTROLLER_G"))
            .withGravityType(GravityTypeValue.Elevator_Static);
    public static final double ELEVATOR_SAFTEY_BUMP = Units.inchesToMeters(2);
  }

  public class ArmConstants {
    // Arm Constants:
    public static final int ARM_MOTOR_ID = 23;
    public static final int ARM_ENCODER_ID = 24;
    public static final double ARM_TARGET_THRESHOLD = 0.25; // In rads
    public static final InvertedValue ARM_FOLLOWER_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final double CORAL_ARM_CRUISE_VELOCITY = 4;
    public static final double CORAL_ARM_ACCELERATION = 1.75;
    public static final double ALGAE_ARM_CRUISE_VELOCITY = 4;
    public static final double ALGAE_ARM_ACCELERATION = 0.65;
    public static final double SAFTEY_ARM_CRUISE_VELOCITY = 2;
    public static final double SAFTEY_ARM_ACCELERATION = 0.30;
    public static final double DANGER_ARM_ANGLE = Units.degreesToRadians(95);
    public static final double ARM_LENGTH =
        Units.inchesToMeters(LOADER.getDoubleValue("arm", "LENGTH_PIVOT_TO_FUNNEL"));
    public static final double ARM_WIDTH =
        Units.inchesToMeters(LOADER.getDoubleValue("arm", "DEPTH_CORAL_POCKET"));
    // ((shaft sprocket / pivot sprocket) / gearbox) * rotations to radians ratio)
    public static final double SENSOR_TO_MECHANISM_RATIO = (1.0 / ((16.0 / 64.0) / 20.0));
    public static final double ARM_FORWARD_LIMT = Units.radiansToRotations(Math.PI);
    public static final double ARM_REVERSE_LIMT =
        Units.radiansToRotations(Units.degreesToRadians(-130));
    public static final Slot0Configs ARM_GAINS =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("arm", "CONTROLLER_P"))
            .withKI(LOADER.getDoubleValue("arm", "CONTROLLER_I"))
            .withKD(LOADER.getDoubleValue("arm", "CONTROLLER_D"))
            .withKS(LOADER.getDoubleValue("arm", "CONTROLLER_S"))
            .withKV(LOADER.getDoubleValue("arm", "CONTROLLER_V"))
            .withKA(LOADER.getDoubleValue("arm", "CONTROLLER_A"))
            .withKG(LOADER.getDoubleValue("arm", "CONTROLLER_G"))
            .withGravityType(GravityTypeValue.Arm_Cosine);
  }

  public static final class PickupConstatns {
    public static final int TIME_OF_FLIGHT_ID = 3;
    public static final int INTAKE_ID = 41;
    public static final int PIVOT_ID = 40;
    public static final double PIVOT_DEPLOYED_ANGLE = 0;
    public static final double PIVOT_STATION_ANGLE = -23;
    public static final double PIVOT_THRESHOLD = 0.5;
    public static final double INTAKE_IN_SPEED = 0.50;
    public static final double INTAKE_OUT_SPEED = -0.50;
    public static final Slot0Configs PICKUP_GAINS =
        new Slot0Configs().withKP(0.5).withKI(0.00).withKD(0.00);
  }

  public class GameStateManagerConstatns {
    public static final double REQUIRED_ROTATION_FOR_ELEVATOR = Units.degreesToRadians(45);
  }
}
