// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

  private final TalonFX driver;
  private final TalonFXConfiguration driverConfig;
  private double speed;
  public boolean running = true;
  boolean reversed;
  boolean slow = false;
  DigitalInput ballSensor;
  
  /** Commands
   * Start/Stop Front
   * Start/Stop Back
   */

  /** Creates a new Conveyor. */
  public Conveyor(int driverCANId, boolean rear) {
    driver = new TalonFX(driverCANId); 
    driverConfig = new TalonFXConfiguration();
    driverConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
    driverConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
    driverConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    driverConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
    driver.getConfigurator().apply(driverConfig);

    running = false;
    speed = ConveyorConstants.conveyorNominalSpeed;

    if (rear) {
      driver.setInverted(true);
      speed = 0.5;
    }

    if (getName() == "FrontConveyor") {
      ballSensor = new DigitalInput(1);
    } else if (getName() == "BackConveyor") {
      ballSensor = new DigitalInput(2);
    }
  }

  public void start() {
    start(1);
  }

  public void start(double dutyCycle) {
    speed = ConveyorConstants.conveyorNominalSpeed * dutyCycle;
    running = true;
  }

  public void stop() {
    running = false;
  }

  public void setForward() {
    reversed = false;
  }

  public void setReversed() {
    reversed = true;
  }

  private void updateOutput() {
    if (running) {
        if (reversed) {
          if (driver.get() > 0) { // we are going forwards, reverse it
            driver.set(-speed);
          }
        } else { 
          if (driver.get() <= 0) { // going backwards, forwards it
            driver.set(speed);
          }
        }
      } else {
        if (driver.get() > 0) {
          driver.set(0);
        }
      }
    }

  @Override
  public void periodic() {
    updateOutput();
  }
}
