package frc.lib;

import monologue.Annotations.Log;
import edu.wpi.first.math.geometry.Translation2d;
import monologue.Logged;

public class TargetData implements Logged {
  @Log.File
  public Translation2d translation;
  @Log.File
  public double X_offset_ = 0;
  @Log.File
  public double Y_offset_ = 0;
  @Log.File
  public String name_;

  public TargetData(Translation2d t, String n) {
    this.translation = t;
    this.name_ = n;
  }

  public TargetData(Translation2d t) {
    this.translation = t;
    this.name_ = "Unset";
  }

  public TargetData() {
    this.name_ = "Unset";
  }

  public String toString() {
    return this.name_;
  }

  public enum ControlType {
    PIVOT,
    EFFECTOR
  }

  /**
   * @return the current Y including the active offset
   */
  public double getY() {
    return translation.getY() + Y_offset_;
  }

  /**
   * Adjusts the Y offset by the supplied increment
   *
   * @param offset increment to adjust the Y offset by
   */
  public void offsetY(double offset) {
    Y_offset_ += offset;
  }

  /**
   * @return the current stored Y offset
   */
  public double getYOffset() {
    return Y_offset_;
  }

  /** Reset the Y offset to 0 */
  public void resetYOffset() {
    Y_offset_ = 0;
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
