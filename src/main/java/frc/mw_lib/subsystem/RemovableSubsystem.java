package frc.mw_lib.subsystem;

import java.util.List;

import frc.mw_lib.util.ConstantsLoader;

public abstract class RemovableSubsystem extends Subsystem {

  private String subsystem_name_;
  private boolean is_enabled_;

  public RemovableSubsystem() {
    subsystem_name_ = this.getClass().getSimpleName();

    // Determine if the subsystem should be enabled
    List<String> subsystems = ConstantsLoader.getInstance().getStringList("subsytems");
    is_enabled_ = subsystems.contains(subsystem_name_);
  }

  public boolean isEnabled(){
    return is_enabled_;
  }
}
