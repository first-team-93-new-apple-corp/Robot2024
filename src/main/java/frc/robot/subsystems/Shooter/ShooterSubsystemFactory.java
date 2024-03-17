package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.IO.ShooterIOReal;
import frc.robot.subsystems.Shooter.IO.ShooterIOSim;

public class ShooterSubsystemFactory {
    public static ShooterSubsystem build(ShooterConstants constants) {
        if (Utils.isSimulation()) {
            return new ShooterSubsystem(new ShooterIOSim(constants));
        } else {
            return new ShooterSubsystem(new ShooterIOReal(constants));
        }        
    }
}
