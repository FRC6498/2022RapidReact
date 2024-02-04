// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterSim {
    private final FlywheelSim physicsSim;
    private final TalonFX shooter;
    private final TalonFXSimState shooterSim;

    public ShooterSim(TalonFX shooter) {
        physicsSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(
            ShooterConstants.flywheelkV, 
            ShooterConstants.flywheelkA
            ), DCMotor.getFalcon500(2), 1
        ); 
        this.shooter = shooter;
        shooterSim = this.shooter.getSimState();
    }

    public void updateSim() {
        // set input voltage so that it will account for voltage sag
        shooterSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // update the wpilib physics model
        physicsSim.setInputVoltage(shooterSim.getMotorVoltage());

        // step time
        physicsSim.update(0.020);

        // update motor sensors, compensate for units
        shooterSim.setRotorVelocity(physicsSim.getAngularVelocityRPM() * 60.0);
    }
}
