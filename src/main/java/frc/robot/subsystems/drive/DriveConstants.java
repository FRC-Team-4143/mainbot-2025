package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.util.ConstantsLoader;
import frc.mw_lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.mw_lib.swerve.SwerveModuleConstants;
import frc.mw_lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.mw_lib.swerve.SwerveModuleConstantsFactory;
import frc.mw_lib.swerve.utility.ModuleType;

public class DriveConstants {

        private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();


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
        public static final double MAX_TRACTOR_BEAM_VELOCITY_SPEED = MAX_DRIVE_SPEED * 0.25;
        public static final double MAX_TRACTOR_BEAM_OMEGA_SPEED = MAX_DRIVE_ANGULAR_RATE * 0.6;
        public static final double TRACTOR_BEAM_ROTATION_THRESHOLD = Units.degreesToRadians(1);
        public static final double TRACTOR_BEAM_TARGET_DISTANCE = Units.inchesToMeters(0.75);
        public static final double TRACTOR_BEAM_SAFETY_DISTANCE = Units.inchesToMeters(34);
    
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
        public static final PIDController X_FOLLOW_TRANSLATION = new PIDController(3.0, 0, 0.1);
    
        public static final PIDController Y_FOLLOW_TRANSLATION = new PIDController(3.0, 0, 0.1);
    
        public static final PIDController FOLLOW_HEADING = new PIDController(3.0, 0, 0.0);
    
        public static final double CENTER_OFFSET_X =
            Units.inchesToMeters(LOADER.getDoubleValue("drive", "com", "CENTER_OFFSET_X"));
    
        public static final double FAILING_TO_REACH_TARGET_DEBOUNCE_TIME = 0.25;
        public static final double FAILING_TO_REACH_TARGET_SPEEDS_TOLERANCE = 0.1;
      }
