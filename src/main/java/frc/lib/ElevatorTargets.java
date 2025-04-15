package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import monologue.Annotations.Log;

public class ElevatorTargets {

  public enum TargetType {
    SAFETY(0.90041, Rotation2d.fromDegrees(-45), "SAFETY"),
    CLIMB(0.90041, Rotation2d.fromDegrees(121.201), "CLIMB"),
    CORAL_INTAKE(0.75, Rotation2d.fromDegrees(-117), "CORAL_INTAKE"),
    L4(2.109, Rotation2d.fromDegrees(-19.599), Rotation2d.fromDegrees(-63), "L4"),
    L3(1.423500, Rotation2d.fromDegrees(-3.515), "L3"),
    L2(1.0235, Rotation2d.fromDegrees(-3.515), "L2"),
    L1(0, Rotation2d.kZero, "L1"),
    L1_FLICK(0, Rotation2d.kZero, "L1_FLICK"),
    ALGAE_STOW(0.90769, Rotation2d.fromDegrees(-12.9199), "ALGAE_STOW"),
    ALGAE_LOW(0.90848, Rotation2d.fromDegrees(-12.9199), "ALGAE_LOW"),
    ALGAE_HIGH(1.35857, Rotation2d.fromDegrees(-12.9199), "ALGAE_HIGH"),
    ALGAE_PROCESSOR(0.7829, Rotation2d.fromDegrees(-40.1660), "ALGAE_PROCESSOR"),
    BARGE(2.08, Rotation2d.fromDegrees(23.3789), "BARGE"),
    STATION(0.90041, Rotation2d.fromDegrees(135), "STATION");

    @Log.File private double elevator_height_ = 0;
    @Log.File private double elevator_offset_ = 0;
    @Log.File private Optional<Rotation2d> staging_arm_angle_ = Optional.empty();
    @Log.File private Rotation2d arm_angle_ = Rotation2d.kZero;
    @Log.File private Rotation2d arm_offset_ = Rotation2d.kZero;
    @Log.File private String name_ = "";

    TargetType(double elevator_height, Rotation2d arm_angle, String name) {
      elevator_height_ = elevator_height;
      arm_angle_ = arm_angle;
      name_ = name;
    }

    TargetType(
        double elevator_height, Rotation2d arm_angle, Rotation2d staging_arm_angle, String name) {
      elevator_height_ = elevator_height;
      arm_angle_ = arm_angle;
      staging_arm_angle_ = Optional.of(staging_arm_angle);
      name_ = name;
    }

    public String toString() {
      return this.name_;
    }

    // Height Methods
    /**
     * Returns the current height including the active offset
     *
     * @return
     */
    public double getHeight() {
      return elevator_height_ + elevator_offset_;
    }

    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      elevator_offset_ += offset;
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public double getHeightOffset() {
      return elevator_offset_;
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      elevator_offset_ = 0;
    }

    // Angle Methods
    /**
     * Returns the current angle including the active offset
     *
     * @return target angle
     */
    public Rotation2d getAngle() {
      return arm_angle_.rotateBy(arm_offset_);
    }

    /**
     * Returns an optional angle for staging while traveling
     *
     * @return
     */
    public Optional<Rotation2d> getStagingAngle() {
      return staging_arm_angle_;
    }

    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(Rotation2d offset) {
      arm_offset_ = arm_offset_.rotateBy(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public Rotation2d getAngleOffset() {
      return arm_offset_;
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      arm_offset_ = new Rotation2d();
    }

    /** Resets both height and angle offsets */
    public void resetOffsets() {
      resetAngleOffset();
      resetHeightOffset();
    }

    /**
     * @return the name of the target
     */
    public String getName() {
      return name_;
    }
  }
}
