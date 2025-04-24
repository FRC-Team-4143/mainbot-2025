package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.mw_lib.util.ConstantsLoader;


import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

        private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();


    // Elevator Constants
    public static final int ELEVATOR_MASTER_ID = 21;
    public static final int ELEVATOR_FOLLOWER_ID = 22;
    public static final double ELEVATOR_TARGET_THRESHOLD = Units.inchesToMeters(1); // In m
    public static final double ELEVATOR_TARGET_THRESHOLD_CLOSE = Units.inchesToMeters(6); // In m
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
    public static final double ELEVATOR_ZERO_THRESHOLD = 0; // In m
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 80.0;
    public static final double ELEVATOR_HEIGHT_PIVOT_MIN =
        Units.inchesToMeters(LOADER.getDoubleValue("elevator", "HEIGHT_PIVOT_MIN"));
    public static final double ELEVATOR_HEIGHT_PIVOT_MAX =
        Units.inchesToMeters(LOADER.getDoubleValue("elevator", "HEIGHT_PIVOT_MAX"))
            - 0.05; // 0.05m of safety
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
    public static final MotionMagicConfigs ELEVATOR_MAGIC_CONFIG =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(300.0)
            .withMotionMagicAcceleration(400.0)
            .withMotionMagicJerk(1000.0);
    public static final double ELEVATOR_SAFETY_BUMP = Units.inchesToMeters(2);
  }
