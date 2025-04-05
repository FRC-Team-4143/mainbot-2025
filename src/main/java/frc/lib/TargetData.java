package frc.lib;

import monologue.Annotations.Log;
import monologue.Logged;

public class TargetData implements Logged {
  @Log.File public double height_;
  @Log.File public double angle_;
  @Log.File public ControlType type_;
  @Log.File public double height_offset_ = 0;
  @Log.File public double angle_offset_ = 0;
  @Log.File public String name_;

  public TargetData(double h, double a, ControlType t, String n) {
    this.height_ = h;
    this.angle_ = a;
    this.type_ = t;
    this.name_ = n;
  }

  public TargetData(double h, double a, ControlType t) {
    this.height_ = h;
    this.angle_ = a;
    this.type_ = t;
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

  // Height Methods
  /**
   * Returns the current height including the active offset
   *
   * @return
   */
  public double getHeight() {
    return height_ + height_offset_;
  }

  /**
   * Adjusts the height offset by the supplied increment
   *
   * @param offset
   */
  public void offsetHeight(double offset) {
    height_offset_ += offset;
  }

  /**
   * Returns the current stored angle offset
   *
   * @return
   */
  public double getHeightOffset() {
    return height_offset_;
  }

  /** Reset the height offset to 0 */
  public void resetHeightOffset() {
    height_offset_ = 0;
  }

  // Angle Methods
  /**
   * Returns the current angle including the active offset
   *
   * @return target angle
   */
  public double getAngle() {
    return angle_ + angle_offset_;
  }

  /**
   * Adjusts the angle offset by the supplied increment
   *
   * @param offset
   */
  public void offsetAngle(double offset) {
    angle_offset_ += offset;
  }

  /**
   * Returns the current stored angle offset
   *
   * @return
   */
  public double getAngleOffset() {
    return angle_offset_;
  }

  /** Resets the angle offset to 0 */
  public void resetAngleOffset() {
    angle_offset_ = 0;
  }

  /**
   * Returns the control type associated with the target
   *
   * @return stored control type
   */
  public ControlType getControlType() {
    return type_;
  }
}
