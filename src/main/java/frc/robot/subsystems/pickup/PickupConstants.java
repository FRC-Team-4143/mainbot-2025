package frc.robot.subsystems.pickup;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class PickupConstants {
  public static final int TIME_OF_FLIGHT_ID = 4;
  public static final int INTAKE_ID = 41;
  public static final int PIVOT_ID = 40;
  public static final double PIVOT_THRESHOLD = 0.5;
  public static final double SENSOR_TO_MECHANISM_RATIO = (85.714 / 1.0);
  public static final Rotation2d PIVOT_OFFSET = Rotation2d.fromDegrees(35);
  public static final Rotation2d PIVOT_DEPLOYED_ANGLE = Rotation2d.fromDegrees(-35);
  public static final Rotation2d PIVOT_STATION_ANGLE = Rotation2d.fromDegrees(59);
  public static final Rotation2d PIVOT_CLIMB_ANGLE = Rotation2d.fromDegrees(0);
  public static final double INTAKE_IN_SPEED = 0.50;
  public static final double INTAKE_OUT_SPEED = -0.50;
  public static final Slot0Configs PICKUP_GAINS =
      new Slot0Configs().withKP(42.857).withKI(0.00).withKD(0.00);
  public static final double STATOR_CURRENT_LIMIT = 80;

  public static final double INTAKE_OFF_SET_Y = -Units.inchesToMeters(8);
  public static final double TOF_CORAL_DISTANCE = Units.inchesToMeters(6.5) * 1000;
}
