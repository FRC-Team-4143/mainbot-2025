package frc.mw_lib.subsystem;

public abstract class RemovableSubsystem extends Subsystem {

  private String subsystem_name_;
  private boolean is_enabled_;

  public RemovableSubsystem() {
    subsystem_name_ = this.getClass().getSimpleName();
    is_enabled_ = SubsystemManager.getEnabledSubsystems().contains(subsystem_name_);
  }

  public boolean isEnabled() {
    return is_enabled_;
  }
}
