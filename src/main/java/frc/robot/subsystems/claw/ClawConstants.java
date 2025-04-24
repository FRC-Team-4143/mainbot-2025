package frc.robot.subsystems.claw;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.mw_lib.util.ConstantsLoader;


public final class ClawConstants {

    private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

    public static final double TIME_OF_FLIGHT_DIST = 50;
    public static final int TIME_OF_FLIGHT_ID = 3;
    public static final int WHEEL_MOTOR_ID = 11;
    public static final double WHEEL_CORAL_SHOOT_SPEED = 0.6;
    public static final double WHEEL_CORAL_BLAST_SPEED = 0.9;
    public static final double WHEEL_ALGAE_SHOOT_SPEED = 0.5;
    public static final double WHEEL_ALGAE_BLAST_SPEED = 0.5;
    public static final double CORAL_LOAD_SPEED = -0.5;
    public static final double ALGAE_LOAD_SPEED = -0.7;
    public static final double ALGAE_IDLE_SPEED = -0.05;
    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final String CORAL_COLOR = new Color(255, 255, 255).toHexString();
    public static final String ALGAE_COLOR = new Color(0, 255, 255).toHexString();
    public static final InvertedValue WHEEL_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;
    public static final double CORAL_IMP_OFFSET = Units.inchesToMeters(LOADER.getDoubleValue("imp", "coral_offset"));
    public static final double ALGAE_IMP_OFFSET = Units.inchesToMeters(LOADER.getDoubleValue("imp", "algae_offset"));
    public static final double CORAL_CURRENT_THRESHOLD = 4.25;
    public static final double ALGAE_SPEED_THRESHOLD = 0.5;
  }
