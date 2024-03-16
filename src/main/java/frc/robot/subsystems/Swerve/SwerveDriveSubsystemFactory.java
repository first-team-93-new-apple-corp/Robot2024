package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants.IntakeConstants;

public class SwerveDriveSubsystemFactory {
    public static SwerveDriveSubsystem build(IntakeConstants constants) {
        if (Utils.isSimulation()) {
            return TunerConstants.DriveTrain;
        } else {
            return TunerConstants.DriveTrain;
        }        
    }
}
