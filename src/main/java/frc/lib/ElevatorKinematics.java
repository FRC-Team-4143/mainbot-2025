package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import frc.mw_lib.util.Util;

public class ElevatorKinematics {
  private double virtual_arm_length_ = 0;
  private double virtual_arm_angle_ = 0;
  private double reachable_max_ = 0;
  private double reachable_min_ = 0;

  public enum SolutionType {
    ABOVE_PIVOT,
    BELOW_PIVOT
  }

  /**
   * Elevator Kinematic's Constructor
   *
   * @param arm_length The Length of the robot's arm in meters
   */
  public ElevatorKinematics(
      double arm_length, double arm_width, double elevator_max, double elevator_min) {
    virtual_arm_length_ = Math.sqrt(Math.pow(arm_length, 2) + Math.pow(arm_width, 2));
    virtual_arm_angle_ = Math.tan(arm_width / arm_length);
    reachable_max_ = elevator_max + virtual_arm_length_;
    reachable_min_ = elevator_min - virtual_arm_length_;
  }

  public double getVirtualArmLength() {
    return virtual_arm_length_;
  }

  public JointSpaceSolution translationToJointSpace(Translation3d t, SolutionType solution_type) {
    double x = t.getX();
    if (Math.abs(x) > virtual_arm_length_) {
      x = Math.copySign(virtual_arm_length_, x);
    }
    double z = t.getZ();
    if (z > reachable_max_) {
      z = reachable_max_;
    }
    if (z < reachable_min_) {
      z = reachable_min_;
    }

    double angle = 0;
    if (solution_type == SolutionType.ABOVE_PIVOT) {
      angle = Math.acos(x / virtual_arm_length_);
    } else {
      angle = -Math.acos(x / virtual_arm_length_);
    }
    double height = z - (Math.sin(angle) * virtual_arm_length_);
    return new JointSpaceSolution(height, angle);
  }

  public Translation3d jointSpaceToTranslation(JointSpaceSolution j) {
    double x = Math.cos(j.getPivotAngle()) * virtual_arm_length_;
    double z = j.getPivotHeight() + (Math.sin(j.getPivotAngle()) * virtual_arm_length_);
    // if (Math.abs(x) > virtual_arm_length_)
    // DataLogManager.log("WARNING: Forward Kinematics X Value : Out of Reach");
    // if (z > reachable_max_)
    // DataLogManager.log("WARNING: Forward Kinematics Z Value : Out of Reach (+)");
    // if (z < reachable_min_)
    // DataLogManager.log("WARNING: Forward Kinematics Z Value : Out of Reach (-)");
    return new Translation3d(x, 0, z);
  }

  public Translation3d jointSpaceToTranslation(double pivot_height, double pivot_angle) {
    return jointSpaceToTranslation(new JointSpaceSolution(pivot_height, pivot_angle));
  }

  public Translation3d constrainReachableTranslation(Translation3d t) {
    return new Translation3d(
        Util.clamp(t.getX(), virtual_arm_length_),
        t.getY(),
        Util.clamp(t.getZ(), reachable_min_, reachable_max_));
  }

  public static class JointSpaceSolution {
    private double pivot_height_ = 0;
    private double pivot_angle_ = 0;

    public JointSpaceSolution(double pivot_height, double pivot_angle) {
      pivot_height_ = pivot_height;
      pivot_angle_ = pivot_angle;
    }

    public String toString() {
      return "pivot_height: " + pivot_height_ + " | pivot_angle: " + pivot_angle_;
    }

    public double getPivotHeight() {
      return pivot_height_;
    }

    public double getPivotAngle() {
      return pivot_angle_;
    }

    public void update(double height, double angle) {
      pivot_height_ = height;
      pivot_angle_ = angle;
    }
  }
}
