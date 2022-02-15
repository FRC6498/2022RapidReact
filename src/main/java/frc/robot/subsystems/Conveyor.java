// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase {

  private final WPI_TalonFX driver;
  private final TalonFXConfiguration driverConfig;

  private final Color cargoColor;
  public boolean running;

  //TODO: How do we use the pico color data?
  //TODO: What LEDs if any?

  /** Commands
   * Start/Stop Front
   * Start/Stop Back
   * Switch ticktock
   * 
   * forward/back/ready ball color
   */

  /** Creates a new Conveyor. */
  public Conveyor(int driverCANId) {
    driver = new WPI_TalonFX(driverCANId);

    driverConfig = new TalonFXConfiguration();
    driverConfig.openloopRamp = 0.5;
    driverConfig.peakOutputForward = 0.1;
    driverConfig.peakOutputReverse = 0.1;
    driverConfig.voltageCompSaturation = 12;

    driver.configAllSettings(driverConfig);
    driver.enableVoltageCompensation(true);

    cargoColor = Color.kGray;
    running = false;
  }

  public void start() {
    start(1);
  }

  public void start(double dutyCycle) {
    driver.set(0.1 * dutyCycle);
  }

  public void stop() {
    driver.stopMotor();
  }

  @Override
  public void periodic() {
    if (driver.get() > 0) {
      // we are moving
      running = true;
    } else {
      running = false;
    }
  }
}
