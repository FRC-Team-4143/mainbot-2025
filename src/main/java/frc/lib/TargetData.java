package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import monologue.Annotations.Log;
import monologue.Logged;

public class TargetData implements Logged {
  @Log.File public Translation3d translation;
  @Log.File public double X_offset_ = 0;
  @Log.File public double Z_offset_ = 0;
  @Log.File public String name_;

  public TargetData(Translation3d t, String n) {
    this.translation = t;
    this.name_ = n;
  }

  public TargetData(Translation3d t) {
    this.translation = t;
    this.name_ = "Unset";
  }

  public TargetData() {
    this.name_ = "Unset";
  }

  public String toString() {
    return this.name_;
  }

  /**
   * @return the current Z including the active offset
   */
  public double getZ() {
    return translation.getZ() + Z_offset_;
  }

  /**
   * Adjusts the Z offset by the supplied increment
   *
   * @param offset increment to adjust the Z offset by
   */
  public void offsetZ(double offset) {
    Z_offset_ += offset;
  }

  /**
   * @return the current stored Z offset
   */
  public double getZOffset() {
    return Z_offset_;
  }

  /** Reset the Z offset to 0 */
  public void resetZOffset() {
    Z_offset_ = 0;
  }

  /**
   * @return the current X including the active offset
   */
  public double getX() {
    return translation.getX() + X_offset_;
  }

  /**
   * Adjusts the X offset by the supplied increment
   *
   * @param offset increment to adjust the X offset by
   */
  public void offsetX(double offset) {
    X_offset_ += offset;
  }

  /**
   * @return the current stored X offset
   */
  public double getXOffset() {
    return X_offset_;
  }

  /** Reset the X offset to 0 */
  public void resetXOffset() {
    X_offset_ = 0;
  }

  /**
   * @return the name of the target
   */
  public String getName() {
    return name_;
  }
}
