package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

import com.pathplanner.lib.auto.NamedCommands;



public class AutoManager {

    // Singleton pattern
    private static AutoManager autoManagerInstance = null;

    public static AutoManager getInstance() {
        if (autoManagerInstance == null) {
            autoManagerInstance = new AutoManager();
        }
        return autoManagerInstance;
    }

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private AutoManager() {
        
        SwerveDrivetrain.getInstance().configurePathPlanner();

        // Register each of the autos
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
