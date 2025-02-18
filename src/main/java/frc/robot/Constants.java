// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.FieldConstants;
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
  public static class Vision {
    public static final String kCameraName = "OV9281-10";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.05, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

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

  public static final class ClawConstants {
    public static final int WHEEL_MOTOR_ID = 11;
    public static final double WHEEL_SHOOT_SPEED = 0.30;
    public static final double WHEEL_LOAD_SPEED = -0.2;
    public static final double ALGAE_IDLE_SPEED = 0.1;
    public static final String CORAL_COLOR = new Color(255, 255, 255).toHexString();
    public static final String ALGAE_COLOR = new Color(0, 255, 255).toHexString();
    public static final InvertedValue WHEEL_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
  }

  public class ClimberConstants {
    public static final int CLIMBER_ID = 35;
    public static final double DEPLOYED_ROTATIONS = 30;
    public static final double RETRACTED_ROTATIONS = 30;
    public static final Slot0Configs CLIMBER_GAINS =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0);
  }

  public class ElevatorConstants {
    // Elevator Constants
    public static final int ELEVATOR_MASTER_ID = 21;
    public static final int ELEVATOR_FOLLOWER_ID = 22;
    public static final int ELEVATOR_LIMIT_SWITCH_PORT_NUMBER = 4;
    public static final double ELEVATOR_TARGET_THRESHOLD = 0.25; // In m
    public static final InvertedValue ELEVATOR_MASTER_INVERSION_ =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ELEVATOR_FOLLOWER_INVERSION =
        InvertedValue.Clockwise_Positive;
    // Units.inchesToMeters(Sprocket Circumference * Math.PI) / gearbox ratio *
    // rigging
    public static final double ELEVATOR_ROTATIONS_TO_METERS =
        Units.inchesToMeters(1.751 * Math.PI) / 4 * 2;
    public static final double ELEVATOR_CRUISE_VELOCITY = 5.0 / ELEVATOR_ROTATIONS_TO_METERS;
    public static final double ELEVATOR_ACCEL = 3.0 / ELEVATOR_ROTATIONS_TO_METERS;
    public static final double ELEVATOR_EXPO_KV = 0.11733;
    public static final double ELEVATOR_EXPO_KA = 0.0070285;
    public static final double ELEVATOR_ZERO_THRESHOLD = 0; // In m
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 40.0;
    public static final double ELEVATOR_HEIGHT_ABOVE_PIVOT = Units.inchesToMeters(8.0);
    public static final double ELEVATOR_MIN_HEIGHT = Units.inchesToMeters(28.25);
    public static final double ELEVATOR_MAX_HEIGHT =
        Units.inchesToMeters(96.76) - ELEVATOR_HEIGHT_ABOVE_PIVOT - 0.1; // 0.1m of safety

    public static final Slot0Configs ELEVATOR_GAINS =
        new Slot0Configs()
            .withKP(1.0)
            .withKI(0.0) // <-DO NOT TOUCH!!!!!!!!!
            .withKD(0.08)
            .withKS(0.06766)
            .withKV(0.11733)
            .withKA(0.0070285)
            .withKG(0.43)
            .withGravityType(GravityTypeValue.Elevator_Static);

    // Arm Constants:
    public static final int ARM_MOTOR_ID = 23;
    public static final int ARM_ENCODER_ID = 24;
    public static final double ARM_TARGET_THRESHOLD = 0.25; // In rads
    public static final InvertedValue ARM_FOLLOWER_INVERSION =
        InvertedValue.CounterClockwise_Positive;
    public static final double ARM_HOME_POSITION = 0;
    public static final double ARM_CRUISE_VELOCITY = 4;
    public static final double ARM_ACCELERATION = 2;
    public static final double ARM_LENGTH = Units.inchesToMeters(12.5);
    // ((shaft sprocket / pivot sprocket) / gearbox) * rotations to radians ratio)
    public static final double SENSOR_TO_MECHANISM_RATIO = (1.0 / ((16.0 / 64.0) / 20.0));
    public static final double ARM_FORWARD_LIMT = Units.radiansToRotations(Math.PI);
    public static final double ARM_REVERSE_LIMT =
        Units.radiansToRotations(Units.degreesToRadians(-95));
    public static final Slot0Configs ARM_GAINS =
        new Slot0Configs()
            .withKP(40.0)
            .withKI(0.0) // DO NOT TOUCH!!!!!!!!!
            .withKD(0.0)
            .withKS(0.024495)
            .withKV(8.778)
            .withKA(0.25148)
            .withKG(0.28)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public enum Target {
      L4(
          FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(13),
          Rotation2d.fromDegrees(130),
          ControlType.EFFECTOR),
      L3(
          FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(125),
          ControlType.EFFECTOR),
      L2(
          FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(125),
          ControlType.EFFECTOR),

      STATION(1.076666, Rotation2d.fromRadians(-1.027767), ControlType.PIVOT),
      CLIMB(ELEVATOR_MIN_HEIGHT, new Rotation2d(), ControlType.PIVOT),
      STOW(0, Rotation2d.fromDegrees(-90), ControlType.PIVOT),
      ALGAE_LOW(
          0.23665818349136578, Rotation2d.fromRadians(2.4942527611020524), ControlType.EFFECTOR),
      ALGAE_HIGH(1.200, Rotation2d.fromDegrees(90 + 33), ControlType.EFFECTOR);

      // BARGE();

      Target(double height, Rotation2d angle, ControlType type) {
        this.angle = angle;
        this.height = height;
        this.type = type;
      }

      public final double height;
      public final Rotation2d angle;

      public enum ControlType {
        PIVOT,
        EFFECTOR
      }

      public final ControlType type;
    }
  }
}
