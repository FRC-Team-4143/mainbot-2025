package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;

public class TargetData implements Logged {
  @Log.File public double height_;
  @Log.File public Rotation2d angle_;
  @Log.File public ControlType type_;
  @Log.File public Optional<Rotation2d> staging_arm_angle_;
  @Log.File public double height_offset_ = 0;
  @Log.File public Rotation2d angle_offset_ = new Rotation2d();

  public TargetData(double h, Rotation2d a, ControlType t, Optional<Rotation2d> stagingArmAngle) {
    this.height_ = h;
    this.angle_ = a;
    this.type_ = t;
    this.staging_arm_angle_ = stagingArmAngle;
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
  public Rotation2d getAngle() {
    return angle_.rotateBy(angle_offset_);
  }

  /**
   * Adjusts the angle offset by the supplied increment
   *
   * @param offset
   */
  public void offsetAngle(Rotation2d offset) {
    angle_offset_ = angle_offset_.rotateBy(offset);
  }

  /**
   * Returns the current stored angle offset
   *
   * @return
   */
  public Rotation2d getAngleOffset() {
    return angle_offset_;
  }

  public Optional<Rotation2d> getStagingArmAngle() {
    return staging_arm_angle_;
  }

  /** Resets the angle offset to 0 */
  public void resetAngleOffset() {
    angle_offset_ = new Rotation2d();
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
