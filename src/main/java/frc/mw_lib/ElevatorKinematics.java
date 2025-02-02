package frc.mw_lib;

public class ElevatorKinematics {
  private double arm_length_ = 0;
  private double arm_z_offset_ = 0;

  /**
   * Elevator Kinematic's Constructor
   *
   * @param arm_length The Length of the robot's arm in meters
   */
  public ElevatorKinematics(double arm_length, double arm_z_off) {
    arm_length_ = arm_length;
    arm_z_offset_ = arm_z_off;
  }

  /**
   * Calculates and returns the z position needed for the wanted z-position MUST HAVE ANGLE SET
   * CORRECT FOR HEIGHT TO BE ACCURATE
   *
   * @param desiredZ the wanted z position of the Claw
   * @param desiredAngle the current angle of the pivot arm
   * @return the needed z for the wanted z position of the elevator
   */
  public double desiredElevatorZ(double desiredZ, double desiredAngle) {
    return desiredZ - calZOffset(desiredAngle) + (Math.cos(desiredAngle) * arm_z_offset_);
  }

  /**
   * Calculates and returns the z position needed for the wanted z-position MUST HAVE ANGLE SET
   * CORRECT FOR HEIGHT TO BE ACCURATE
   *
   * @param desiredZ the wanted z position of the Claw
   * @param desiredAngle the current angle of the pivot arm
   * @return the needed z for the wanted z position of the elevator
   */
  public double effectorZ(double current_z, double current_angle) {
    return current_z + calZOffset(current_angle) - (Math.cos(current_angle) * arm_z_offset_);
  }

  /**
   * Calculates and returns the pivot arm angle needed for the wanted x-position of the claw
   *
   * @param desiredX The wanted x position of the Claw
   * @param isRightSide True for the angle to go right and False for left
   * @return the joint angle needed for the desired x position
   */
  public double desiredJointAngle(double desiredX) {
    if (desiredX > arm_length_) {
      desiredX = arm_length_;
    }
    if (desiredX <= 0) {
      return calAngleWithX(desiredX);
    }
    return calAngleWithX(desiredX);
  }

  public double calXOffset(double angle) {
    return arm_length_ * Math.cos(angle);
  }

  public double calZOffset(double angle) {
    return arm_length_ * Math.sin(angle);
  }

  public double calAngleWithX(double X) {
    return Math.acos(X / arm_length_);
  }

  public double calAngleWithZ(double Z) {
    return Math.asin(Z / arm_length_);
  }
}
