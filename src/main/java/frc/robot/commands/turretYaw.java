package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret;

public class turretYaw extends CommandBase {
    
Turret turret;
    public  void turretTurn(Turret turret) {
        addRequirements(turret);
        turret.setDefaultCommand(new RunCommand(()->
        turret.turretTurn(),
        turret));
        
    }
}
