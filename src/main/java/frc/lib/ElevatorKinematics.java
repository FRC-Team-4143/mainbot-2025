package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

public class ElevatorKinematics {
  private double arm_length_ = 0;
  private double arm_width_ = 0;
  private double virtual_arm_length;
  private double virtual_arm_angle;

  /**
   * Elevator Kinematic's Constructor
   *
   * @param arm_length The Length of the robot's arm in meters
   */
  public ElevatorKinematics(double arm_length, double arm_width_) {
    this.arm_length_ = arm_length;
    this.arm_width_ = arm_width_;
    this.virtual_arm_length = Math.sqrt(Math.pow(arm_length, 2) + Math.pow(arm_width_, 2));
    this.virtual_arm_angle = Math.tan(arm_width_ / arm_length);
  }

  public JointSpaceTarget translationToJointSpace(Translation2d t) {
    JointSpaceTarget target = new JointSpaceTarget();
    target.pivot_angle = -Math.acos(t.getX() / virtual_arm_length);
    target.pivot_height = -(Math.sin(target.pivot_angle) * virtual_arm_length) + t.getY();
    return target;
  }

  public Translation2d jointSpaceToTranslation(JointSpaceTarget j) {
    double x = Math.cos(j.pivot_angle) * virtual_arm_length;
    double y = j.pivot_height + (Math.sin(j.pivot_angle) * virtual_arm_length);
    return new Translation2d(x, y);
  }

  public Translation2d jointSpaceToTranslation(double pivot_height, double pivot_angle) {
    return jointSpaceToTranslation(new JointSpaceTarget(pivot_height, pivot_angle));
  }

  public static class JointSpaceTarget {
    public double pivot_height = 0;
    public double pivot_angle = 0;

    public JointSpaceTarget(double height, double angle) {
      this.pivot_height = height;
      this.pivot_angle = angle;
    }

    public JointSpaceTarget() {}

    public String toString() {
      return "pivot_height: " + pivot_height + " |pivot_angle: " + pivot_angle;
    }
  }
}
