package frc.mw_lib.swerve.utility;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.mw_lib.util.ConstantsLoader;
import java.util.Hashtable;

public class ModuleType {
  public final String name;
  public final double steerRatio;
  public final double driveRatio;

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  ModuleType(String name, double steerRatio, double driveRatio) {
    this.name = name;
    this.steerRatio = steerRatio;
    this.driveRatio = driveRatio;
  }

  private static final ModuleType[] ALL_TYPES = {
    // MK4I
    // https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module
    new ModuleType("MK4I-L1", 150.0 / 7.0, 8.14),
    new ModuleType("MK4I-L2", 150.0 / 7.0, 6.75),
    new ModuleType("MK4I-L3", 150.0 / 7.0, 6.12),

    // MK4I Adapter 16T Pinion Kit
    // https://www.swervedrivespecialties.com/products/kit-adapter-16t-drive-pinion-gear-mk4i
    new ModuleType("MK4I-L1+", 150.0 / 7.0, 7.13),
    new ModuleType("MK4I-L2+", 150.0 / 7.0, 5.9),
    new ModuleType("MK4I-L3+", 150.0 / 7.0, 5.36),

    // MK4N
    // https://www.swervedrivespecialties.com/collections/kits/products/mk4n-swerve-module
    new ModuleType("MK4N-L1+", 18.75, 7.13),
    new ModuleType("MK4N-L2+", 18.75, 5.9),
    new ModuleType("MK4N-L3+", 18.75, 5.36),

    // MKC
    // https://www.swervedrivespecialties.com/collections/kits/products/mk4c-swerve-module
    new ModuleType("MK4C-L1+", 12.8, 7.13),
    new ModuleType("MK4C-L2+", 12.8, 5.9),
    new ModuleType("MK4C-L3+", 12.8, 5.36),

    // MK4
    // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    new ModuleType("MK4-L1", 12.8, 8.14),
    new ModuleType("MK4-L2", 12.8, 6.75),
    new ModuleType("MK4-L3", 12.8, 6.12),
    new ModuleType("MK4-L4", 12.8, 5.14)
  };

  public static Hashtable<String, ModuleType> ALL_MODULE_TYPES = new Hashtable<>();

  static {
    for (ModuleType type : ALL_TYPES) {
      ALL_MODULE_TYPES.put(type.name, type);
    }
  }

  /**
   * @param position String represneting the module location [fl, fr, bl, br, etc..]
   * @return ModuleType to be load the gear ratio constants from
   */
  public static ModuleType getModuleType(String position) {
    String type = LOADER.getStringValue("drive", position, "MODULE_TYPE");
    String gearing = LOADER.getStringValue("drive", position, "MODULE_GEARING");
    ModuleType module = ALL_MODULE_TYPES.get(type + "-" + gearing);
    if (module == null) {
      DataLogManager.log(
          "ERROR: Invalid Module Type: " + type + "-" + gearing + "\nDefaulting to MK4I-L3");
      module = ALL_MODULE_TYPES.get("MK4I-L3");
    }
    return module;
  }
}
