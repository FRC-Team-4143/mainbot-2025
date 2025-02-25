package frc.mw_lib.swerve.utility;

import java.util.Hashtable;

public class ModuleType {
    public final String name;
    public final double steerRatio;
    public final double driveRatio;

    ModuleType(String name, double steerRatio, double driveRatio) {
        this.name = name;
        this.steerRatio = steerRatio;
        this.driveRatio = driveRatio;
    }

    // MK4I
    public static final ModuleType MK4I_L1 = new ModuleType("MK4I-L1", 150.0 / 7.0, 8.14);
    public static final ModuleType MK4I_L2 = new ModuleType("MK4I-L2", 150.0 / 7.0, 6.75);
    public static final ModuleType MK4I_L3 = new ModuleType("MK4I-L3", 150.0 / 7.0, 6.12);
    public static final ModuleType MK4I_L4 = new ModuleType("MK4I-L4", 150.0 / 7.0, 5.14);
    public static final ModuleType MK4I_L1_PLUS = new ModuleType("MK4I-L1", 150 / 7.0, 7.13);
    public static final ModuleType MK4I_L2_PLUS = new ModuleType("MK4I-L2", 150 / 7.0, 5.9);
    public static final ModuleType MK4I_L3_PLUS = new ModuleType("MK4I-L3", 150 / 7.0, 5.36);

    // MK4N
    public static final ModuleType MK4N_L1_PLUS = new ModuleType("MK4N-L1+", 18.75, 7.13);
    public static final ModuleType MK4N_L2_PLUS = new ModuleType("MK4N-L2+", 18.75, 5.9);
    public static final ModuleType MK4N_L3_PLUS = new ModuleType("MK4N-L3+", 18.75, 5.36);

    private static ModuleType[] ALL_TYPES = {
    MK4I_L1,
    MK4I_L2,
    MK4I_L3,
    MK4I_L4,
    MK4N_L1_PLUS,
    MK4N_L2_PLUS,
    MK4N_L3_PLUS
  };


    public static Hashtable<String, ModuleType> ALL_MODULE_TYPES = new Hashtable<>();

    static {
        for (ModuleType module : ALL_TYPES) {
            ALL_MODULE_TYPES.put(module.name, module);
        }
    }
    
}
