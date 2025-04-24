package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.util.ConstantsLoader;

public class ArmConstants {

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  // Arm Constants:
  public static final int ARM_MOTOR_ID = 23;
  public static final int ARM_ENCODER_ID = 24;
  public static final double ARM_TARGET_THRESHOLD = 0.25; // In rads
  public static final InvertedValue ARM_FOLLOWER_INVERSION = InvertedValue.Clockwise_Positive;
  public static final SensorDirectionValue ABSOLUTE_ENCODER_INVERSION =
      SensorDirectionValue.Clockwise_Positive;
  // ((shaft sprocket / pivot sprocket) / gearbox) * rotations to radians ratio)
  public static final double SENSOR_TO_MECHANISM_RATIO = (1.0 / ((16.0 / 64.0) / 20.0));
  public static final double ARM_FORWARD_LIMIT = Units.radiansToRotations(30);
  public static final double ARM_REVERSE_LIMIT = Units.degreesToRotations(-275);
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
  public static final MotionMagicConfigs ARM_MAGIC_CONFIG =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(1.5)
          .withMotionMagicAcceleration(10.0)
          .withMotionMagicJerk(30.0);
}
