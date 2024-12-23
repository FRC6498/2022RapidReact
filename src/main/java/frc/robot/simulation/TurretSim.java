// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

/** Add your docs here. */
public class TurretSim {
    private final DCMotorSim turretPhysicsSim;
    private final TalonFX motor;
    private TalonFXSimState motorSim;

    private final MutAngle turretAngle = Rotations.zero().mutableCopy();
    private final MutAngularVelocity turretSpeed = RotationsPerSecond.zero().mutableCopy();

    public TurretSim(TalonFX turretMotor) {
        motor = turretMotor;
        motorSim = motor.getSimState();
        turretPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.kV, 
                TurretConstants.kA
            ), 
            DCMotor.getFalcon500(1), 
            TurretConstants.turretRotorToMechanismRatio
        );
    }

    public double getTurretSimPos() {
        return turretPhysicsSim.getAngularPositionRotations();
    }

    public double getTurretSimVel() {
        return turretPhysicsSim.getAngularVelocityRPM() / 60.0;
    }

    public void updateSim() {
        motorSim = motor.getSimState();

        // set input voltage so that it will account for voltage sag
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // update the wpilib physics model
        turretPhysicsSim.setInputVoltage(motorSim.getMotorVoltage() - TurretConstants.kS);

        // step time
        turretPhysicsSim.update(0.020);

        // update motor sensors, compensate for units
        turretSpeed.mut_replace(turretPhysicsSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        turretAngle.mut_replace(turretPhysicsSim.getAngularPositionRotations(), Rotations);
        motorSim.setRotorVelocity(turretSpeed.in(RotationsPerSecond));
        motorSim.setRawRotorPosition(turretAngle.in(Rotations));
    }
}
