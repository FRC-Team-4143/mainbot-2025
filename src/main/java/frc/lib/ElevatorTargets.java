package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.TargetData.ControlType;
import frc.robot.Constants.ElevatorConstants;
import java.util.Optional;

public class ElevatorTargets {
  public static Optional<Rotation2d> ReefStagingAngle = Optional.of(Rotation2d.fromDegrees(90));

  public enum Target {
    L4(
        new TargetData(
            FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
            Rotation2d.fromDegrees(145.3),
            ControlType.EFFECTOR,
            ReefStagingAngle)),
    L3(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR,
            ReefStagingAngle)),
    L2(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR,
            ReefStagingAngle)),
    L1(
        new TargetData(
            FieldConstants.ReefHeight.L1.HEIGHT + Units.inchesToMeters(6),
            Rotation2d.fromDegrees(-1.027767),
            ControlType.EFFECTOR,
            ReefStagingAngle)),
    STATION(
        new TargetData(
            0.8468 + Units.inchesToMeters(3),
            Rotation2d.fromRadians(-1.027767),
            ControlType.EFFECTOR,
            Optional.empty())),
    CLIMB(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0),
            ControlType.PIVOT,
            Optional.empty())),
    STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(-90),
            ControlType.PIVOT,
            Optional.empty())),
    CORAL_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(90),
            ControlType.PIVOT,
            Optional.empty())),
    ALGAE_LOW(
        new TargetData(
            0.9702231159054557 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(149),
            ControlType.EFFECTOR,
            Optional.empty())),
    ALGAE_HIGH(
        new TargetData(
            1.2535345791562702 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(130.79),
            ControlType.EFFECTOR,
            Optional.empty())),
    ALGAE_PROCESSOR(
        new TargetData(
            0.6381 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(-55),
            ControlType.EFFECTOR,
            Optional.empty())),
    BARGE(
        new TargetData(
            2.159 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(90),
            ControlType.EFFECTOR,
            Optional.empty())),
    ALGAE_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(90),
            ControlType.PIVOT,
            Optional.empty()));

    Target(TargetData td) {
      this.td = td;
    }

    // Height Methods
    /**
     * Returns the current height including the active offset
     *
     * @return
     */
    public double getHeight() {
      return td.getHeight();
    }

    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      td.offsetHeight(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public double getHeightOffset() {
      return td.getHeightOffset();
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      td.resetHeightOffset();
    }

    // Angle Methods
    /**
     * Returns the current angle including the active offset
     *
     * @return target angle
     */
    public Rotation2d getAngle() {
      return td.getAngle();
    }

    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(Rotation2d offset) {
      td.offsetAngle(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public Rotation2d getAngleOffset() {
      return td.getAngleOffset();
    }

    public Optional<Rotation2d> getStagingArmAngle() {
      return td.getStagingArmAngle();
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      td.resetAngleOffset();
    }

    /**
     * Returns the control type associated with the target
     *
     * @return stored control type
     */
    public ControlType getControlType() {
      return td.type_;
    }

    public TargetData getLoggingObject() {
      return td;
    }

    private TargetData td;
  }
}
