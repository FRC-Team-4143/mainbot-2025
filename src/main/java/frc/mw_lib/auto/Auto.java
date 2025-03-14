package frc.mw_lib.auto;

import java.util.ArrayList;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoManager;

public class Auto extends SequentialCommandGroup {

  private static final AutoFactory auto_factory_ = AutoManager.getInstance().getAutoFactory();
  public static final SendableChooser<Command> auto_chooser_ = AutoManager.getInstance().getAutoChooser();

  private String name_ = "Auto";
  private ArrayList<Trajectory> trajectory_list_ = new ArrayList<>();

  Auto(String name){}

  public ArrayList<Trajectory> getTrajectories(){
    return trajectory_list_;
  }
  
  private void registerAuto(){
    auto_chooser_.addOption(name_, this);
  }
  
}
