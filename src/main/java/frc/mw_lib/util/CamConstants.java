package frc.mw_lib.util;

import edu.wpi.first.math.geometry.Transform3d;

public class CamConstants {
  public String camera_name;

  public Transform3d camera_transform;

  public CamConstants(String n, Transform3d t) {
    this.camera_name = n;
    this.camera_transform = t;
  }
}
