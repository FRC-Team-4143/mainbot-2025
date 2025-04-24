package frc.robot.subsystems.pose_estimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.mw_lib.util.CamConstants;
import frc.mw_lib.util.ConstantsLoader;
import frc.mw_lib.util.TagLayouts;
import java.util.List;

public class PhotonVisionConstants {

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  public static final List<CamConstants> CAMERAS = LOADER.getCameras("vision");

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout TAG_LAYOUT =
      TagLayouts.getTagLayoutFromPath("apriltagLayouts/onlyReef.json");

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2.0, 2.0, 4);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
}
