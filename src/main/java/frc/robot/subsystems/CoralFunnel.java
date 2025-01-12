// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.FeederConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class CoralFunnel extends Subsystem {

    private SparkFlex left_feeder_motor_;
    private SparkFlex right_feeder_motor_;

    public enum FeedingMode {
        IDLE,
        FEEDING,
        SCORING
    }

    // Singleton pattern
    private static CoralFunnel example_instance = null;

    // C
    public static CoralFunnel getInstance() {
        if (example_instance == null) {
            example_instance = new CoralFunnel();
        }
        return example_instance;
    }

    /**
     * Class Members
     */
    private CoralFunnelPeriodicIo io_;

    private CoralFunnel() {
        // Create io object first in subsystem configuration
        io_ = new CoralFunnelPeriodicIo();
        left_feeder_motor_ = new SparkFlex(FeederConstants.LEFT_FEEDER_MOTOR, MotorType.kBrushless);
        right_feeder_motor_ = new SparkFlex(FeederConstants.RIGHT_FEEDER_MOTOR, MotorType.kBrushless);

        SparkFlexConfig config_ = new SparkFlexConfig();
        config_.inverted(FeederConstants.LEFT_FEEDER_INVERTED);
        left_feeder_motor_.configure(config_, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        config_.inverted(FeederConstants.RIGHT_FEEDER_INVERTED);
        right_feeder_motor_.configure(config_, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // Call reset last in subsystem configuration
        reset();
    }

    /**
     * This function should be logic and code to fully reset your subsystem.
     * This is called during initialization, and should handle I/O configuration and
     * initializing data members.
     */
    @Override
    public void reset() {

    }

    /**
     * Inside this function, all of the SENSORS should be read into variables
     * stored in the PeriodicIO class defined below. There should be no calls
     * to output to actuators, or any logic within this function.
     */
    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.left_current_sensor_ = left_feeder_motor_.getOutputCurrent();
        io_.right_current_sensor_ = right_feeder_motor_.getOutputCurrent();
    }

    /**
     * Inside this function, all of the LOGIC should compute updates to output
     * variables in the PeriodicIO class defined below. There should be no
     * calls to read from sensors or write to actuators in this function.
     */
    @Override
    public void updateLogic(double timestamp) {
        switch (io_.feeding_mode_) {
            case FEEDING:
                io_.feeder_output_ = FeederConstants.FEEDER_SPEED;
                break;
            case SCORING:
                io_.feeder_output_ = FeederConstants.SCORE_SPEED;
                break;
            case IDLE:
            default:
                io_.feeder_output_ = FeederConstants.IDLE_SPEED;
                break;
        }

        io_.average_motor_current_ = (io_.left_current_sensor_ + io_.right_current_sensor_) / 2;

    }

    /**
     * Inside this function actuator OUTPUTS should be updated from data contained
     * in the PeriodicIO class
     * defined below. There should be little to no logic contained within this
     * function, and no sensors
     * should be read.
     */
    @Override
    public void writePeriodicOutputs(double timestamp) {
        left_feeder_motor_.set(io_.feeder_output_);
        right_feeder_motor_.set(-io_.feeder_output_);

    }

    /**
     * Inside this function telemetry should be output to smartdashboard. The data
     * should be collected out
     * of the PeriodicIO class instance defined below. There should be no sensor
     * information read in this function nor
     * any outputs made to actuators within this function. Only publish to
     * smartdashboard here.
     */
    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("CURRENT", io_.average_motor_current_);
    }

    public class CoralFunnelPeriodicIo implements Logged {
        @Log.File
        public double feeder_output_ = 0;
        @Log.File
        public double left_current_sensor_ = 0;
        @Log.File
        public double right_current_sensor_ = 0;
        @Log.File
        public FeedingMode feeding_mode_ = FeedingMode.IDLE;
        @Log.File
        public double average_motor_current_ = 0;

    }

    public void setFeedingMode(FeedingMode mode) {
        io_.feeding_mode_ = mode;
    }

    public boolean hasCoral() {
        return (io_.average_motor_current_ > 45);
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}