package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorKinematics {
  private double arm_length_ = 0;
  private double arm_width_ = 0;
  private double virtual_arm_length_ = 0;
  private double virtual_arm_angle_ = 0;

  /**
   * Elevator Kinematic's Constructor
   *
   * @param length The Length of the robot's arm in meters
   */
  public ElevatorKinematics(double length, double width) {
    arm_length_ = length;
    arm_width_ = width;
    virtual_arm_length_ = Math.sqrt(Math.pow(length, 2) + Math.pow(width, 2));
    virtual_arm_angle_ = Math.tan(width / length);
    System.out.println("Arm Length: " + Units.metersToInches(virtual_arm_length_));
  }

  public JointSpaceTarget translationToJointSpace(Translation3d t) {
    JointSpaceTarget target = new JointSpaceTarget();
    target.pivot_angle = -Math.acos(t.getX() / virtual_arm_length_);
    target.pivot_height = -(Math.sin(target.pivot_angle) * virtual_arm_length_) + t.getZ();
    return target;
  }

  public Translation3d jointSpaceToTranslation(JointSpaceTarget j) {
    double x = Math.cos(j.pivot_angle) * virtual_arm_length_;
    double z = j.pivot_height + (Math.sin(j.pivot_angle) * virtual_arm_length_);
    if (Math.abs(x) > virtual_arm_length_)
      System.out.println("WARNING: Kinematics X Value : Out of Reach");
    if (z > ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX + virtual_arm_length_)
      System.out.println("WARNING: Kinematics Z Value : Out of Reach (+)");
    if (z < ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN - virtual_arm_length_)
      System.out.println("WARNING: Kinematics Z Value : Out of Reach (-)");
    return new Translation3d(x, 0, z);
  }

  public Translation3d jointSpaceToTranslation(double pivot_height, double pivot_angle) {
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
      return "pivot_height: " + pivot_height + " | pivot_angle: " + pivot_angle;
    }
  }
}
