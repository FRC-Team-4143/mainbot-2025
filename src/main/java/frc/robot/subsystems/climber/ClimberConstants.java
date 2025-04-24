package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class ClimberConstants {
    public static final int STRAP_ID = 31;
    public static final int PRONG_ID = 1;
    public static final int PRONG_ID_A = 8;
    public static final int PRONG_ID_B = 9;
    public static final int ARM_ID = 2;
    public static final InvertedValue STRAP_INVERSION = InvertedValue.Clockwise_Positive;
    public static final Slot0Configs STRAP_GAINS =
        new Slot0Configs().withKP(0.16).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0);
    public static final double PRONG_DEPLOY_SPEED = 0.4;
    public static final double PRONG_HOLD_SPEED = -0.4;
    public static final double ARM_DEPLOY_SPEED = 0.2;
    public static final double ARM_HOLD_SPEED = 0.08;
    public static final double ARM_SUPPLY_CURRENT_LIMIT = 10;
    public static final double STRAP_RETRACTED_POSITION = 80;
    public static final double STRAP_SETPOINT_BUMP = (STRAP_RETRACTED_POSITION / 25.0);
    public static final double PRONG_PRESET_COUNT = 17;
    public static final double DEPLOYING_TIME = 1.2;
    public static final double PRONG_P = 0.040;
    public static final double PRONG_D = 0.002;
  }
