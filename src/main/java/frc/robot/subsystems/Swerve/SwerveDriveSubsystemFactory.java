package frc.robot.subsystems.Swerve;


import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstants2023;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstants2024;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstantsSIM;

public class SwerveDriveSubsystemFactory {
    public static SwerveDriveSubsystem build(SwerveDriveConstants constants) {
        if (constants.DriveBase == Constants.BotName.Tobor27) {
            return TunerConstants2024.DriveTrain;
        } else if (constants.DriveBase == Constants.BotName.Tobor26) {
            return TunerConstants2023.DriveTrain;
        } else if (constants.DriveBase.equals(Constants.BotName.SIM)) {
            return TunerConstantsSIM.DriveTrain;
        } else {
            return TunerConstantsSIM.DriveTrain;
        }
        
    }
}
