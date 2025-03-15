package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.mw_lib.auto.Auto;
import frc.robot.autos.*;
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

  public void registerAutos() {
    // Register Autos
    registerAuto(J4_LeftStation_L4.class);
    registerAuto(J4_LeftStation_L4_LeftStation_K4.class);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", auto_chooser_);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(auto_chooser_.getSelected());
  }

  /**
   * Takes the supplied class type and adds a new instance of it to the auto chooser list
   *
   * @param <T>
   * @param clazz class to register to auto chooser
   */
  public <T> void registerAuto(Class<T> clazz) {
    try {
      auto_chooser_.addOption(clazz.getSimpleName(), (Auto) clazz.newInstance());
    } catch (InstantiationException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }

  /**
   * Retrieves the auto factory instance in AutoManager
   *
   * @return global auto factory instance
   */
  public AutoFactory getAutoFactory() {
    return auto_factory_;
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
