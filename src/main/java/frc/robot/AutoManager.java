package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoManager {
  public final AutoFactory autoFactory;
  public final SendableChooser<Command> autoChooser;
  // Singleton pattern
  private static AutoManager autoManagerInstance = null;

  public static AutoManager getInstance() {
    if (autoManagerInstance == null) {
      autoManagerInstance = new AutoManager();
    }
    return autoManagerInstance;
  }

  private AutoManager() {
    autoFactory =
        new AutoFactory(
            PoseEstimator.getInstance()::getRobotPose,
            PoseEstimator.getInstance()::setRobotOdometry,
            SwerveDrivetrain.getInstance()::setTargetSample,
            true, // enables auto flipping
            SwerveDrivetrain.getInstance());
    // Create the auto chooser
    autoChooser = new SendableChooser<Command>();

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.getSelected());
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  public SendableChooser<Command> getAutoChooser(){
    return autoChooser;
  }
}
