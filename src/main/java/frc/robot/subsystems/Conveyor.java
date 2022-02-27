// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase implements Loggable {

  private final WPI_TalonFX driver;
  private final DigitalInput ballSensor;
  private final TalonFXConfiguration driverConfig;

  private Color cargoColor;
  public boolean running;
  @Log
  public boolean empty;
  //@Log.BooleanBox(name = "Ball Present (Color)")
  boolean colorEmpty;
  //@Log.BooleanBox(name = "Ball Present (Current)")
  boolean currentEmpty;
  public int colorSensorId;
  private ColorMatch colorMatch;
  @Log
  private double driverOutput;
  DoubleSolenoid tickTock;
  

  //TODO: What LEDs if any?

  /** Commands
   * Start/Stop Front
   * Start/Stop Back
   */

  /** Creates a new Conveyor. */
  public Conveyor(int driverCANId, int colorSensorId, int beambreakId) {
    driver = new WPI_TalonFX(driverCANId);

    driverConfig = new TalonFXConfiguration();
    driverConfig.openloopRamp = 0.5;
    driverConfig.peakOutputForward = 0.5;
    driverConfig.peakOutputReverse = -0.5;
    driverConfig.voltageCompSaturation = 12;

    driver.configAllSettings(driverConfig);
    driver.enableVoltageCompensation(true);
    driverOutput = 0.0;

    cargoColor = Color.kGray;
    running = false;
    ballSensor = new DigitalInput(beambreakId);

    this.colorSensorId = colorSensorId;
    colorMatch = new ColorMatch();
    colorMatch.setConfidenceThreshold(0.95);
    colorMatch.addColorMatch(Color.kRed);
    colorMatch.addColorMatch(Color.kBlue);
  }

  public void start() {
    start(-1);
  }

  public void start(double dutyCycle) {
    driverOutput = Constants.ConveyorConstants.conveyorNominalSpeed * dutyCycle;
    running = true;
  }

  public void stop() {
    driverOutput = 0;
  }

  public void reverse() {
    driverOutput *= -1;
  }

  public Color getCargoColor() {
    return cargoColor;
  }

  public void setCargoColor(Color color) {
    cargoColor = color;
  }

  public boolean isBallPresent(boolean color) {
    if (color) {
      // get color match
      ColorMatchResult match = colorMatch.matchClosestColor(cargoColor);
      if (match.color == Color.kRed || match.color == Color.kBlue) {
        colorEmpty = true;
      } else colorEmpty = false;

      // get current match
      if (driver.getStatorCurrent() > ballPresentCurrentThreshold) {
        currentEmpty = true;
      } else currentEmpty = false;
      
      return colorEmpty || currentEmpty;
    } else {
      // use beam break
      return ballSensor.get();
    }
  }

  @Override
  public void periodic() {
    // are we empty
    empty = !ballSensor.get();

    if (empty) {
      driver.set(0);
    } else {
      driver.set(0.1);
    }
  }
}

// flow:
// check if conveyor is empty
// check ball color if conveyor is full
// check if shooter is ready

// if the shooter is ready and the conveyor is full and the ball is the correct color
//    run the conveyor until the feeder takes the ball
// if the intake is full and the conveyor is empty
//    run the intake slowly and the conveyor until the conveyor is full
