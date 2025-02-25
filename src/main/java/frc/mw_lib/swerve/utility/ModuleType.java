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

    private static final ModuleType[] ALL_TYPES = {
        // MK4I
        new ModuleType("MK4I-L1", 150.0 / 7.0, 8.14),
        new ModuleType("MK4I-L2", 150.0 / 7.0, 6.75),
        new ModuleType("MK4I-L3", 150.0 / 7.0, 6.12),
        new ModuleType("MK4I-L4", 150.0 / 7.0, 5.14),
        new ModuleType("MK4I-L1", 150 / 7.0, 7.13),
        new ModuleType("MK4I-L2", 150 / 7.0, 5.9),
        new ModuleType("MK4I-L3", 150 / 7.0, 5.36),

        // MK4N
        new ModuleType("MK4N-L1+", 18.75, 7.13),
        new ModuleType("MK4N-L2+", 18.75, 5.9),
        new ModuleType("MK4N-L3+", 18.75, 5.36),

        // MK4
        new ModuleType("MK4N-L1+", 18.75, 7.13),
        new ModuleType("MK4N-L2+", 18.75, 5.9),
        new ModuleType("MK4N-L3+", 18.75, 5.36)
    };

    public static Hashtable<String, ModuleType> ALL_MODULE_TYPES = new Hashtable<>();

    static {
        for (ModuleType type : ALL_TYPES) {
            ALL_MODULE_TYPES.put(type.name, type);
        }
    }
    
}
