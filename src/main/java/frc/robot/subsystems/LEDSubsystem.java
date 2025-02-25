// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import monologue.Annotations.Log;
import monologue.Logged;

public class LEDSubsystem extends Subsystem {

  // Singleton pattern
  private static LEDSubsystem led_instance_ = null;

  public static LEDSubsystem getInstance() {
    if (led_instance_ == null) {
      led_instance_ = new LEDSubsystem();
    }
    return led_instance_;
  }

  public enum LEDMode {
    SCORE_READY,
    LOST,
    CAGE,
    PARTY,
    REEF_FACE_0,
    REEF_FACE_1,
    REEF_FACE_2,
    REEF_FACE_3,
    REEF_FACE_4,
    REEF_FACE_5,
    DEFENSE
  }

  private AddressableLED led_1_ = new AddressableLED(LEDConstants.LED_PORT_1);
  private AddressableLED led_2_ = new AddressableLED(LEDConstants.LED_PORT_2);
  private AddressableLEDBuffer led_buffer_1_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH_1);
  private AddressableLEDBuffer led_buffer_2_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH_2);

  /** Class Members */
  private LEDPeriodicIo io_;

  Debouncer pickupNoteDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private EventLoop led_eventloop_ = new EventLoop();

  private LEDSubsystem() {
    // Create io object first in subsystem configuration
    io_ = new LEDPeriodicIo();

    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      led_buffer_1_.setRGB(i, 0, 255, 0);
    }
    for (int i = 0; i < led_buffer_2_.getLength(); i++) {
      led_buffer_2_.setRGB(i, 0, 255, 0);
    }
    led_2_.setLength(led_buffer_2_.getLength());
    led_2_.setData(led_buffer_2_);
    led_2_.start();
    led_1_.setLength(led_buffer_1_.getLength());
    led_1_.setData(led_buffer_1_);
    led_1_.start();

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is
   * called during
   * initialization, and should handle I/O configuration and initializing data
   * members.
   */
  @Override
  public void reset() {
    io_ = new LEDPeriodicIo();
    led_buffer_1_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH_1);
    led_buffer_2_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH_2);
    led_1_.setLength(led_buffer_1_.getLength());
    led_1_.setData(led_buffer_1_);
    led_1_.start();
    led_2_.setLength(led_buffer_2_.getLength());
    led_2_.setData(led_buffer_2_);
    led_2_.start();
  }

  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any
   * logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors
   * or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    if (Claw.getInstance().isCoralMode()) {
      io_.led_team_str_ = 255;
    } else {
      io_.led_team_str_ = 255 / 2;
    }

    if (io_.isCriticalError) {
      io_.led_cycle_length_ = 5;
      if (io_.led_cycle_state) {
        io_.pattern = LEDPattern.steps(Map.of(0, Color.kOrange, 0.5, Color.kRed));

      } else {
        io_.pattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kOrange));
      }
    } else {
      if (!DriverStation.isDisabled()) {
        switch (io_.led_mode_) {
          case SCORE_READY:
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
              scoreModeLED(0, 0, io_.led_team_str_);
            } else {
              scoreModeLED(io_.led_team_str_, 0, 0);
            }
            setCyleLength(0);
            break;
          case PARTY:
            partyModeLED();
            setCyleLength(25);
            break;
          case REEF_FACE_0:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kDarkRed, Color.kRed);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case REEF_FACE_1:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kDarkOrange, Color.kOrange);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case REEF_FACE_2:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kLightYellow);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case REEF_FACE_3:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kDarkGreen, Color.kGreen);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case REEF_FACE_4:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kDarkBlue, Color.kBlue);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case REEF_FACE_5:
            io_.pattern = LEDPattern.progressMaskLayer(
                () -> Elevator.getInstance().getCurrentHeight()
                    / ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            io_.color = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous, Color.kPurple, Color.kLavender);
            io_.pattern = io_.color.mask(io_.pattern);
            break;
          case CAGE:
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
              io_.color = LEDPattern.solid(Color.kBlue);
              
            } else {
              io_.color = LEDPattern.solid(Color.kRed);
            }
            io_.pattern = io_.color.blink(Seconds.of(0.5));
            break;
          case DEFENSE:
            defenseModeLED();

            setCyleLength(60);
            break;
          case LOST:
          default:
            io_.pattern = LEDPattern.solid(Color.kBlack);
            break;
        }
      } else {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          io_.pattern = LEDPattern.solid(Color.kBlue);
        } else {
          io_.pattern = LEDPattern.solid(Color.kRed);
        }
      }
    }

    io_.pattern.applyTo(led_buffer_1_);
    io_.pattern.applyTo(led_buffer_2_);
    ledCycle();
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in the PeriodicIO
   * class defined below. There should be little to no logic contained within this
   * function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    led_1_.setData(led_buffer_1_);
    led_2_.setData(led_buffer_1_);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor
   * information read
   * in this function nor any outputs made to actuators within this function. Only
   * publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
  }

  public class LEDPeriodicIo implements Logged {
    @Log.File
    public boolean led_cycle_state = true;
    @Log.File
    public int led_cycle_counter = 25;
    @Log.File
    public LEDMode led_mode_ = LEDMode.SCORE_READY;
    @Log.File
    public int led_cycle_length_ = 25;
    @Log.File
    public int led_team_str_ = 255;
    @Log.File
    public boolean isCriticalError = false;

    @Log.File
    LEDPattern pattern = LEDPattern.progressMaskLayer(
        () -> Elevator.getInstance().getCurrentHeight() / ElevatorConstants.ELEVATOR_MAX_HEIGHT);

    @Log.File
    LEDPattern color = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kPurple, Color.kLavender);
  }

  public void setIfCritcalError(boolean isError) {
    io_.isCriticalError = isError;
  }

  public void scoreModeLED(int r, int g, int b) {
    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      if (io_.led_cycle_state) {
        if (i % 2 == 0) {
          led_buffer_1_.setRGB(i, r, g, b);
        } else {
          led_buffer_1_.setRGB(i, b, g, r);
        }
      } else {
        if (i % 2 == 1) {
          led_buffer_1_.setRGB(i, r, g, b);
        } else {
          led_buffer_1_.setRGB(i, b, g, r);
        }
      }
    }
    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      if (io_.led_cycle_state) {
        if (i % 2 == 0) {
          led_buffer_2_.setRGB(i, r, g, b);
        } else {
          led_buffer_2_.setRGB(i, 0, 0, 0);
        }
      } else {
        if (i % 2 == 1) {
          led_buffer_2_.setRGB(i, r, g, b);
        } else {
          led_buffer_2_.setRGB(i, 0, 0, 0);
        }
      }
    }
  }

  public void idleModeLED(int r, int g, int b) {
    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      if (i < Constants.LEDConstants.LED_LENGTH_1
          * (Constants.LEDConstants.LED_LENGTH_1 / Elevator.getInstance().getCurrentHeight())) {
        led_buffer_1_.setRGB(i, r, g, b);
      } else {
        led_buffer_1_.setRGB(i, 0, 0, 0);
      }
    }

    for (int i = 0; i < led_buffer_2_.getLength(); i++) {
      if (i < Constants.LEDConstants.LED_LENGTH_2
          * (Constants.LEDConstants.LED_LENGTH_2 / Elevator.getInstance().getCurrentHeight())) {
        led_buffer_2_.setRGB(i, r, g, b);
      } else {
        led_buffer_2_.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void partyModeLED() {
    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      led_buffer_1_.setRGB(
          i, (int) (Math.random() * 255), (int) (Math.random() * 255), (int) (Math.random() * 255));
    }
    for (int i = 0; i < led_buffer_2_.getLength(); i++) {
      led_buffer_2_.setRGB(
          i, (int) (Math.random() * 255), (int) (Math.random() * 255), (int) (Math.random() * 255));
    }
  }

  public void defenseModeLED() {
    if (io_.led_cycle_counter > 70) {
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 255);
      }
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 0, 0, 0);
      }
    } else if (io_.led_cycle_counter > 60) {
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 255, 0, 0);
      }
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 0);
      }

    } else if (io_.led_cycle_counter > 50) {
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 255);
      }
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 0, 0, 0);
      }

    } else if (io_.led_cycle_counter > 40) {
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 255, 0, 0);
      }
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 0);
      }
      io_.led_cycle_state = true;

    } else if (io_.led_cycle_state) {
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 255);
      }
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 0, 0, 0);
      }
      io_.led_cycle_state = !io_.led_cycle_state;
    } else {
      for (int i = 0; i < led_buffer_2_.getLength(); i++) {
        led_buffer_2_.setRGB(i, 255, 0, 0);
      }
      for (int i = 0; i < led_buffer_1_.getLength(); i++) {
        led_buffer_1_.setRGB(i, 0, 0, 0);
      }
      io_.led_cycle_state = !io_.led_cycle_state;
    }

    // for (int i = 0; i < led_buffer_1_.getLength(); i++) {
    // led_buffer_1_.setRGB(i, , ,);
    // }
    // for (int i = 0; i < led_buffer_2_.getLength(); i++) {
    // led_buffer_2_.setRGB(i, , ,);
    // }
  }

  public void setColorTeam() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setColorRGBCycle(0, 0, 255, io_.led_cycle_state);
    } else {
      setColorRGBCycle(255, 0, 0, io_.led_cycle_state);
    }
  }

  public void setCyleLength(int length) {
    io_.led_cycle_length_ = length;
    io_.led_cycle_counter = length;
  }

  public void setColorRGB(int r, int g, int b) {
    for (int i = 0; i < led_buffer_1_.getLength(); i++) {
      led_buffer_1_.setRGB(i, r, g, b);
    }
    for (int i = 0; i < led_buffer_2_.getLength(); i++) {
      led_buffer_2_.setRGB(i, r, g, b);
    }
  }

  public void setColorRGBUpperCycle(int r, int g, int b) {
    for (int i = 0; i < led_buffer_1_.getLength(); i += 2) {
      led_buffer_1_.setRGB(i, r, g, b);
    }
    for (int i = 0; i < led_buffer_2_.getLength(); i += 2) {
      led_buffer_2_.setRGB(i, r, g, b);
    }
  }

  public void setColorRGBLowerCycle(int r, int g, int b) {
    for (int i = 1; i < led_buffer_1_.getLength(); i += 2) {
      led_buffer_1_.setRGB(i, r, g, b);
    }
    for (int i = 1; i < led_buffer_2_.getLength(); i += 2) {
      led_buffer_2_.setRGB(i, r, g, b);
    }
  }

  public void setColorRGBCycle(int r, int g, int b, boolean cycle) {
    if (cycle) {
      setColorRGBUpperCycle(r, g, b);
    } else {
      setColorRGBLowerCycle(r, g, b);
    }
  }

  public void setColorRGBFlash(int r, int g, int b, boolean cycle) {
    if (cycle) {
      setColorRGB(r, g, b);
    } else {
      setColorRGB(0, 0, 0);
    }
  }

  public void ledCycle() {
    io_.led_cycle_counter--;
    if (io_.led_cycle_counter <= 0) {
      io_.led_cycle_state = !io_.led_cycle_state;
      io_.led_cycle_counter = io_.led_cycle_length_;
    }
  }

  public void setLEDMode(LEDMode led_mode) {
    io_.led_mode_ = led_mode;
  }

  public class LEDSubsystemPeriodicIo implements Logged {
    public boolean led_cycle_state = true;
    public int led_cycle_counter = 25;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
