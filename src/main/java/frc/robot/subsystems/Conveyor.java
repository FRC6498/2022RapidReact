// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  WPI_TalonFX rearDriver;
  WPI_TalonFX frontDriver;
  TalonFXConfiguration driverConfig;

  Color frontCargo;
  Color rearCargo;
  Color preFeederCargo;

  //TODO: What is the ticktock?
  //TODO: What is the ready to shoot sensor type?
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
  public Conveyor() {
    rearDriver = new WPI_TalonFX(Constants.ConveyorConstants.rearDriverCANId);
    frontDriver = new WPI_TalonFX(Constants.ConveyorConstants.frontDriverCANId);

    driverConfig = new TalonFXConfiguration();
    driverConfig.openloopRamp = 0.5;
    driverConfig.peakOutputForward = 0.1;
    driverConfig.peakOutputReverse = 0.1;
    driverConfig.voltageCompSaturation = 12;

    rearDriver.configAllSettings(driverConfig);
    frontDriver.configAllSettings(driverConfig);
    rearDriver.enableVoltageCompensation(true);
    frontDriver.enableVoltageCompensation(true);
  }

  public void startRear() {
    rearDriver.set(0.1);
  }

  public void stopRear() {
    rearDriver.stopMotor();
  }

  public void startFront() {
    frontDriver.set(0.1);
  }

  public void stopFront() {
    frontDriver.stopMotor();
  }

  public void stopConveyor() {
    stopFront();
    stopRear();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
