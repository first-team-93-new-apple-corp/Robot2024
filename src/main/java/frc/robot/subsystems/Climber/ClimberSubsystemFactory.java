package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Climber.IO.ClimberIOReal;
import frc.robot.subsystems.Climber.IO.ClimberIOSim;

public class ClimberSubsystemFactory {
    public static ClimberSubsystem build(ClimberConstants constants, XboxController op) {
        if (Utils.isSimulation()) {
            return new ClimberSubsystem(new ClimberIOSim(constants, op));
        } else {
            return new ClimberSubsystem(new ClimberIOReal(constants, op));
        }        
    }
}
