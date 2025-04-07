package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;

public class ElevatorKinematics {
  private double virtual_arm_length_ = 0;
  private double virtual_arm_angle_ = 0;
  private double reachable_max_ = 0;
  private double reachable_min_ = 0;

  /**
   * Elevator Kinematic's Constructor
   *
   * @param arm_length The Length of the robot's arm in meters
   */
  public ElevatorKinematics(double arm_length, double arm_width, double elevator_max, double elevator_min) {
    virtual_arm_length_ = Math.sqrt(Math.pow(arm_length, 2) + Math.pow(arm_width, 2));
    virtual_arm_angle_ = Math.tan(arm_width / arm_length);
    reachable_max_ = elevator_max + virtual_arm_length_;
    reachable_min_ = elevator_min - virtual_arm_length_;
  }

  public JointSpaceTarget translationToJointSpace(Translation3d t) {
    JointSpaceTarget target = new JointSpaceTarget();
    double x = t.getX();
    if(Math.abs(x) > virtual_arm_length_){
      System.out.println("WARNING: Inverse Kinematics X Value : Out of Reach");
      x = Math.copySign(virtual_arm_length_, x);
    }
    double z = t.getZ();
    if(z > reachable_max_){
      System.out.println("WARNING: Inverse Kinematics Z Value : Out of Reach (+)");
      z = reachable_max_;
    }
    if(z < reachable_min_){
      System.out.println("WARNING: Inverse Kinematics Z Value : Out of Reach (-)");
      z = reachable_min_;
    }
    
    target.pivot_angle = -Math.acos(x / virtual_arm_length_);
    target.pivot_height = -(Math.sin(target.pivot_angle) * virtual_arm_length_) + z;
    return target;
  }

  public Translation3d jointSpaceToTranslation(JointSpaceTarget j) {
    double x = Math.cos(j.pivot_angle) * virtual_arm_length_;
    double z = j.pivot_height + (Math.sin(j.pivot_angle) * virtual_arm_length_);
    if (Math.abs(x) > virtual_arm_length_)
      System.out.println("WARNING: Forward Kinematics X Value : Out of Reach");
    if (z > reachable_max_)
      System.out.println("WARNING: Forward Kinematics Z Value : Out of Reach (+)");
    if (z < reachable_min_)
      System.out.println("WARNING: Forward Kinematics Z Value : Out of Reach (-)");
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
