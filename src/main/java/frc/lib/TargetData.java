package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import monologue.Annotations.Log;
import monologue.Logged;

public class TargetData implements Logged {
  @Log.File public Translation3d base_translation;
  @Log.File public Translation3d current_translation;
  @Log.File public String name_;

  public TargetData(Translation3d t, String n) {
    this.base_translation = t;
    this.name_ = n;
    current_translation = base_translation;
  }

  public TargetData(Translation3d t) {
    this.base_translation = t;
    this.name_ = "Unset";
  }

  public TargetData() {
    this.name_ = "Unset";
  }

  public String toString() {
    return this.name_;
  }

  public Translation3d getTranslation() {
    return current_translation;
  }

  public void offSet(Translation3d t) {
    current_translation = current_translation.plus(t);
  }

  public void resetOffsets() {
    current_translation = base_translation;
  }

  /**
   * @return the name of the target
   */
  public String getName() {
    return name_;
  }
}
