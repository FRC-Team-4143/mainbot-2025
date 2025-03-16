package frc.mw_lib.auto;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoManager {
  // Singleton pattern
  private static AutoManager autoManagerInstance = null;

  public static AutoManager getInstance() {
    if (autoManagerInstance == null) {
      autoManagerInstance = new AutoManager();
    }
    return autoManagerInstance;
  }

  public final AutoFactory auto_factory_;
  public final SendableChooser<Auto> auto_chooser_;

  private AutoManager() {
    auto_factory_ =
        new AutoFactory(
            PoseEstimator.getInstance()::getRobotPose,
            PoseEstimator.getInstance()::setRobotOdometry,
            SwerveDrivetrain.getInstance()::setTargetSample,
            true, // enables auto flipping
            SwerveDrivetrain.getInstance());
    // Create the auto chooser
    auto_chooser_ = new SendableChooser<Auto>();
    // Set default option to not move and wait 15 seconds
    auto_chooser_.setDefaultOption("Do_Nothing", new Do_Nothing());
    // Bind a callback on selected change to display auto
    auto_chooser_.onChange((auto) -> displaySelectedAuto(auto));
  }

  public void registerAutos(Auto... autos) {
    for (Auto auto : autos) {
      auto_chooser_.addOption(auto.getClass().getSimpleName(), (Auto) auto);
    }
    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", auto_chooser_);
  }

  /**
   * Retrieves the auto factory instance in AutoManager
   *
   * @return global auto factory instance
   */
  public AutoFactory getAutoFactory() {
    return auto_factory_;
  }

  public Auto getSelectedAuto() {
    Auto auto = auto_chooser_.getSelected();
    DataLogManager.log("Selected auto routine: " + auto.getClass().getSimpleName());
    return auto;
  }

  /**
   * Displays the selected auto as a trajectory on smart dashboard
   *
   * @param selected_auto new auto to display
   */
  private void displaySelectedAuto(Auto selected_auto) {
    PoseEstimator.getInstance()
        .getFieldWidget()
        .getObject("Selected Auto")
        .setPoses(selected_auto.getPath());
  }

  /** Removes the Selected Auto object */
  public void removeDisplayedAuto() {
    PoseEstimator.getInstance().getFieldWidget().getObject("Selected Auto").setPoses();
  }
}
