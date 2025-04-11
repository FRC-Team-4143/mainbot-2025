package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation3d;

public class Waypoint {
  public Translation3d translation;
  public boolean required;

  public Waypoint(Translation3d t, boolean r) {
    this.translation = t;
    this.required = r;
  }

  public Waypoint(Translation3d t) {
    this.translation = t;
    this.required = false;
  }

  public Waypoint(double x, double y, double z) {
    this.translation = new Translation3d(x, y, z);
    this.required = false;
  }

  public Waypoint(double x, double y, double z, boolean r) {
    this.translation = new Translation3d(x, y, z);
    this.required = r;
  }

  public Waypoint() {
    this.translation = new Translation3d();
    this.required = false;
  }
}
