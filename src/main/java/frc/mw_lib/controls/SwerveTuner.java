package frc.mw_lib.controls;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveTuner{

    private TalonFXTuner driveTuner;
    private TalonFXTuner steerTuner;

    public SwerveTuner (TalonFX dr_motor, TalonFX[] dr_followers, String dr_system_name, TalonFX st_motor, TalonFX[] st_followers, String st_system_name, Subsystem subsystem){

        driveTuner = new TalonFXTuner(dr_motor, dr_followers, dr_system_name, subsystem);

        steerTuner = new TalonFXTuner(st_motor, st_followers, st_system_name, subsystem);
        
    }

}
