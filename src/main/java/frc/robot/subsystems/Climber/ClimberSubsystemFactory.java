package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.IO.ClimberIOReal;
import frc.robot.subsystems.Climber.IO.ClimberIOSim;

public class ClimberSubsystemFactory {
    public static ClimberSubsystem build(ClimberConstants constants) {
        if (Utils.isSimulation()) {
            return new ClimberSubsystem(new ClimberIOSim(constants));
        } else {
            return new ClimberSubsystem(new ClimberIOReal(constants));
        }        
    }
}
