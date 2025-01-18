// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.lib.swerve.SwerveModuleConstants;
import frc.lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.lib.swerve.SwerveModuleConstantsFactory;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean IS_COMP_BOT = Preferences.getBoolean("RobotIsComp", true);

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public class DrivetrainConstants {

        // Can bus names for each of the swerve modules
        public static final String[] MODULE_CANBUS_NAME = { "can0", "can0", "can0", "can0" };

        // Can bus ID for the pigeon
        public static final int PIGEON2_ID = 0;

        // Both sets of gains need to be tuned to your individual robot
        // The steer motor uses MotionMagicVoltage control
        private static final Slot0Configs STEER_GAINS = new Slot0Configs().withKP(100).withKI(0).withKD(0).withKS(0)
                .withKV(0).withKA(0);
        // When using closed-loop control, the drive motor uses:
        // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
        // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(12.5)
                .withKI(0.0)
                .withKD(0.01) // 7 : updated to 3 RJS
                .withKS(0.2)
                .withKV(0.12)
                .withKA(0.05); // 2.4 : updated to 0 RJS

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double SLIP_CURRENT_AMPS = 95.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        private static final double SPEED_AT_12V_MPS = 5.0;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 3.5;

        private static final double DRIVE_GEAR_RATIO = 5.36827799; // L3: 6.12, L2: 5.14 //4.572
        private static final double STEER_GEAR_RATIO = 12.8; // Mk4i: (150.0/7.0), Mk4: 12.8
        private static final double WHEEL_RADIUS_INCH = 1.88; // 1.6090288; // 1.59997;

        private static final boolean STEER_MOTOR_REVERSED = false;
        private static final boolean INVERT_LEFT_DRIVE = false;
        private static final boolean INVERT_RIGHT_DRIVE = false; // true;

        private static final double CHASSIS_WIDTH = 19.0;
        private static final double CHASSIS_LENGTH = 18.0;

        public static final double MAX_DRIVE_SPEED = 1; // 6 meters per second desired top speed
        public static final double MAX_DRIVE_ANGULAR_RATE = Math.PI * 2; // Rotation per second max angular velocity
        public static final double CRAWL_DRIVE_SPEED = 0.4;
        public static final double MAX_TARGET_SPEED = 1;

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(WHEEL_RADIUS_INCH)
                .withSlipCurrent(SLIP_CURRENT_AMPS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSpeedAt12VoltsMps(SPEED_AT_12V_MPS)
                .withFeedbackSource(
                        SteerFeedbackType.None) // .withFeedbackSource(SteerFeedbackType.FusedCANcoder) CRH: Removed
                // for AnalogEncoders
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC);

        // Front Left
        private static final int FLD_MOTOR_ID = 1;
        private static final int FLS_MOTOR_ID = 2;
        private static final int FLS_ENCODER_ID = 0;
        private static final double FLS_ENCODER_OFFSET = 0;

        private static final double FL_X_POS_INCH = CHASSIS_WIDTH / 2;
        private static final double FL_Y_POS_INCH = CHASSIS_LENGTH / 2;

        // Front Right
        private static final int FRD_MOTOR_ID = 3;
        private static final int FRS_MOTOR_ID = 4;
        private static final int FRS_ENCODER_ID = 1;
        private static final double FRS_ENCODER_OFFSET = 0;

        private static final double FR_X_POS_INCH = CHASSIS_WIDTH / 2;
        private static final double FR_Y_POS_INCH = -CHASSIS_LENGTH / 2;

        // Back Left
        private static final int BLD_MOTOR_ID = 5;
        private static final int BLS_MOTOR_ID = 6;
        private static final int BLS_ENCODER_ID = 2;
        private static final double BLS_ENCODER_OFFSET = 0.0;

        private static final double BL_X_POS_INCH = -CHASSIS_WIDTH / 2.;
        private static final double BL_Y_POS_INCH = CHASSIS_LENGTH / 2;

        // Back Right
        private static final int BRD_MOTOR_ID = 7;
        private static final int BRS_MOTOR_ID = 8;
        private static final int BRS_ENCODER_ID = 3;
        private static final double BRS_ENCODER_OFFSET = 0;

        private static final double BR_X_POS_INCH = -CHASSIS_WIDTH / 2.;
        private static final double BR_Y_POS_INCH = -CHASSIS_LENGTH / 2;

        public static final SwerveModuleConstants FL_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                FLS_MOTOR_ID,
                FLD_MOTOR_ID,
                FLS_ENCODER_ID,
                FLS_ENCODER_OFFSET,
                Units.inchesToMeters(FL_X_POS_INCH),
                Units.inchesToMeters(FL_Y_POS_INCH),
                INVERT_LEFT_DRIVE);
        public static final SwerveModuleConstants FR_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                FRS_MOTOR_ID,
                FRD_MOTOR_ID,
                FRS_ENCODER_ID,
                FRS_ENCODER_OFFSET,
                Units.inchesToMeters(FR_X_POS_INCH),
                Units.inchesToMeters(FR_Y_POS_INCH),
                INVERT_RIGHT_DRIVE);
        public static final SwerveModuleConstants BL_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                BLS_MOTOR_ID,
                BLD_MOTOR_ID,
                BLS_ENCODER_ID,
                BLS_ENCODER_OFFSET,
                Units.inchesToMeters(BL_X_POS_INCH),
                Units.inchesToMeters(BL_Y_POS_INCH),
                INVERT_LEFT_DRIVE);
        public static final SwerveModuleConstants BR_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
                BRS_MOTOR_ID,
                BRD_MOTOR_ID,
                BRS_ENCODER_ID,
                BRS_ENCODER_OFFSET,
                Units.inchesToMeters(BR_X_POS_INCH),
                Units.inchesToMeters(BR_Y_POS_INCH),
                INVERT_RIGHT_DRIVE);
    }

    public class ElevatorConstants {

        public static final int ELEVATOR_MASTER_ID = 21;
        public static final int ELEVATOR_FOLLOWER_ID = 22;
        public static final int ELEVATOR_LIMIT_SWITCH_PORT_NUMBER = 4;
        public static final int ARM_MOTOR_ID = 23;
        public static final int ARM_ENCODER_ID = 24;
        public static final double ARM_TARGET_THRESHOLD = 0.25; // In rads
        public static final double ELEVATOR_TARGET_THRESHOLD = 0.25; // In m
        public static final InvertedValue ELEVATOR_MASTER_INVERSION_ = InvertedValue.Clockwise_Positive; 
        public static final InvertedValue ELEVATOR_FOLLOWER_INVERSION_ = InvertedValue.CounterClockwise_Positive; 
        public static final InvertedValue ARM_FOLLOWER_INVERSION_ = InvertedValue.Clockwise_Positive; 
        public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATION = 0;
        public static final double ELEVATOR_CRUISE_VELOCITY = 0;
        public static final double ELEVATOR_ACCELERATION = 0;
        public static final double ELEVATOR_EXPO_KV = 0;
        public static final double ELEVATOR_EXPO_KA = 0;
        public static final double LIMIT_SWITCH_THRESHOLD = 0; // In m
        public static final double ARM_HOME_POSITION = 0;
        public static final double ARM_SENSOR_TO_MECHANISM_RATION = 0;
        public static final double ARM_CRUISE_VELOCITY = 0;
        public static final double ARM_ACCELERATION = 0;
        public static final double ARM_EXPO_KV = 0;
        public static final double ARM_EXPO_KA = 0;
        public static final double ROTOR_TO_CENSOR_RATIO = 1;

        public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKI(0.0) // DO NOT TOUCH!!!!!!!!!
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.0)
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final Slot0Configs ARM_GAINS = new Slot0Configs()
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
