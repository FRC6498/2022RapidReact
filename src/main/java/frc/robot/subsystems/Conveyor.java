// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

  private final WPI_TalonFX driver;
  private final TalonFXConfiguration driverConfig;
  private double speed;
  public boolean running = true;
  boolean reversed;
  DigitalInput ballSensor;
  Vision vision;

  
  /** Commands
   * Start/Stop Front
   * Start/Stop Back
   */

  /** Creates a new Conveyor. */
  public Conveyor(int driverCANId, Vision vision) {
    driver = new WPI_TalonFX(driverCANId); 
    this.vision = vision;

    driverConfig = new TalonFXConfiguration();
    driverConfig.openloopRamp = 0.5;
    driverConfig.peakOutputForward = 1.0;
    driverConfig.peakOutputReverse = -1.0;
    driverConfig.voltageCompSaturation = 12;
    driver.configAllSettings(driverConfig);
    driver.enableVoltageCompensation(true);

    running = false;
    speed = ConveyorConstants.conveyorNominalSpeed;

    if (driverCANId == 9) {
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
    running = true;
  }

  public void start(double dutyCycle) {
    //driverOutput = Constants.ConveyorConstants.conveyorNominalSpeed * dutyCycle;
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
