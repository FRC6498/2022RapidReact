// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.InterpolatingTable;

public class Flywheel extends SubsystemBase implements Loggable {
  // States: [flywheel velocity in rpm]
  // Inputs: [flywheel voltage]
  // Outputs: [velocity]

  // Hardware
  private final CANSparkMax flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  private final WPI_TalonFX hoodRollers;
  // Software
  private boolean flywheelActive;
  @Log
  public double flywheelSpeedSetpoint = -3000.0;
  @Log
  private double speedOffset = 0.0;
  @Log
  private double hoodTargetRPM = 0.0;
  private TalonFXConfiguration hoodConfig;
  private final double nominalVoltage = 12.3;
  private final double flywheelMOI = 0.009803592;
  private final LinearSystem<N1, N1, N1> hoodPlant; // "N1, N1, N1" just means there is 1 state, 1 input, and 1 output (in that order)
  private final LinearSystem<N1, N1, N1> flywheelPlant;
  private final KalmanFilter<N1, N1, N1> hoodObserver;
  private final KalmanFilter<N1, N1, N1> flywheelObserver;
  private final LinearQuadraticRegulator<N1, N1, N1> hoodLQR;
  private final LinearQuadraticRegulator<N1, N1, N1> flywheelLQR;
  private final LinearSystemLoop<N1, N1, N1> hoodLoop;
  private final LinearSystemLoop<N1, N1, N1> flywheelLoop;
  //private double feedforwardOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  //@Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  private ShooterMode mode;
  public Flywheel() {
    // state space
    // hood
    hoodPlant = LinearSystemId.identifyVelocitySystem(nominalVoltage / 6380.0, 0);
    hoodObserver = new KalmanFilter<>(
      Nat.N1(), // states
      Nat.N1(), // outputs
      hoodPlant, // plant
      VecBuilder.fill(3.0), // state standard deviation, or how accurate we think our model is (how close is our predicted state to our real state). In this case it is in RPM
      VecBuilder.fill(0.2), // measurement standard deviation
      0.020
    );
    hoodLQR = new LinearQuadraticRegulator<>(
      hoodPlant, 
      VecBuilder.fill(8.0), // q, velocity error tolerance in rpm. (lower q = more aggressive, as a lower tolerance = velocity must stay in tighter bounds)
      VecBuilder.fill(12.0), // r, control effort tolerance in volts. max voltage that can be sent to the motor, so lower tolerance = lower aggression. 12.0 is pretty good because its close to the battery max voltage
      0.020
    );
    hoodLoop = new LinearSystemLoop<>(hoodPlant, hoodLQR, hoodObserver, 12.0, 0.020);
    //flywheel
    flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), flywheelMOI, 1);
    flywheelObserver = new KalmanFilter<>(
      Nat.N1(), 
      Nat.N1(), 
      flywheelPlant, 
      VecBuilder.fill(3.0), // state std dev (rad/s)
      VecBuilder.fill(0.2), //
      0.020
    );
    flywheelLQR = new LinearQuadraticRegulator<>(
      flywheelPlant, 
      VecBuilder.fill(8.0), // max error tolerance (rad/s) (createFlywheelSystem uses rad/s and units must be consistent)
      VecBuilder.fill(12.0), // max control effort (volts)
      0.020
    );
    flywheelLoop = new LinearSystemLoop<>(flywheelPlant, flywheelLQR, flywheelObserver, 12.0, 0.020);
    // old stuff
    mode = ShooterMode.DISABLED;
    flywheelMotor = new CANSparkMax(ShooterConstants.flywheelCANId, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();
    hoodRollers = new WPI_TalonFX(ShooterConstants.hoodRollerCANId);
    hoodConfig = new TalonFXConfiguration();
    // once we hit 40A for >=100ms, hold at 40A
    hoodConfig.voltageCompSaturation = nominalVoltage;
    hoodConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 100);
    hoodRollers.configAllSettings(hoodConfig);
    hoodRollers.setInverted(TalonFXInvertType.Clockwise);
    flywheelMotor.enableVoltageCompensation(nominalVoltage);
    hoodRollers.enableVoltageCompensation(true);
  }
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  //@Config(name = "Set Flywheel Speed(RPM)")
  @Config
  public void setFlywheelSpeed(double velocity) {
    flywheelSpeedSetpoint = -velocity;
    
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  @Config
  public void setHoodRollerOffset(double rpmOffset) {
    speedOffset = rpmOffset;
  }

  @Log
  public double getHoodSpeed() {
    return falconTicksToRPM(hoodRollers.getSelectedSensorVelocity());
  }

  //@Log(name = "Flywheel Velocity (RPM)")
  @Log
  public double getFlywheelSpeed() {
    return flywheelEncoder.getVelocity();
  }

  @Log
  public double getHoodError() {
    return getHoodSpeed() - hoodTargetRPM;
  }

  public boolean getActive() {
    return mode != ShooterMode.HOMING;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed() - flywheelSpeedSetpoint) < ShooterConstants.flywheelSetpointToleranceRPM;
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
    switch (mode) {
      case MANUAL_FIRE:
      case REJECT:
        flywheelActive = true;
        break;
      case DISABLED:
      case HOMING:
        flywheelActive = false;
        break;
      default:
      flywheelActive = false;
        break;
    }
  }

  private double rpmToNativeUnits(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  private double falconTicksToRPM(double ticks) {
    return ticks / 2048.0 * 600.0;
  }

  @Override
  public void periodic() {
    switch (mode) {
      case MANUAL_FIRE:
        // update setpoints
        flywheelLoop.setNextR(InterpolatingTable.get(distanceToHub).rpm);
        hoodLoop.setNextR(Math.abs(flywheelSpeedSetpoint) + InterpolatingTable.get(distanceToHub).hoodSpeedOffset);
        break;
      case REJECT:
        flywheelLoop.setNextR(1000);
        hoodLoop.setNextR(Math.abs(flywheelLoop.getNextR(0)));
      default:
        break;
    }
    hoodTargetRPM = hoodLoop.getNextR(0);
    flywheelSpeedSetpoint = flywheelLoop.getNextR(0);
    
    // update filters
    flywheelLoop.correct(VecBuilder.fill(getFlywheelSpeed()));
    hoodLoop.correct(VecBuilder.fill(getHoodSpeed()));

    // update LQRs
    flywheelLoop.predict(0.020);
    hoodLoop.predict(0.020);

    if (flywheelActive) {
      flywheelMotor.setVoltage(flywheelLoop.getU(0));
      hoodRollers.setVoltage(hoodLoop.getU(0));
    } else {
      flywheelLoop.setNextR(0);
      hoodLoop.setNextR(0);
      flywheelMotor.stopMotor();
    }
  }

}
