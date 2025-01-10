package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoManager {
  // Singleton pattern
  private static AutoManager autoManagerInstance = null;
  public final AutoFactory autoFactory;

  public static AutoManager getInstance() {
    if (autoManagerInstance == null) {
      autoManagerInstance = new AutoManager();
    }
    return autoManagerInstance;
  }

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private AutoManager() {
    autoFactory =
        new AutoFactory(
            PoseEstimator.getInstance()::getFieldPose,
            PoseEstimator.getInstance()::setRobotOdometry,
            SwerveDrivetrain.getInstance()::followTrajectory,
            true, // enables auto flipping
            SwerveDrivetrain.getInstance());
    // Register each of the autos
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
