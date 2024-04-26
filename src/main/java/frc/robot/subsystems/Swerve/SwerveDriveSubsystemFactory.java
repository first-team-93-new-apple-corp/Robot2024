package frc.robot.subsystems.Swerve;


import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstants2023;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstants2024;
import frc.robot.subsystems.Swerve.TunerConstants.TunerConstantsSIM;

public class SwerveDriveSubsystemFactory {
    private static TunerConstants2023 tunerConstants2023;
    private static TunerConstantsSIM tunerConstantsSIM;
    private static TunerConstants2024 tunerConstants2024;
    public static SwerveDriveSubsystem build(SwerveDriveConstants constants) {

        if (constants.DriveBase == Constants.BotName.Tobor27) {
            tunerConstants2024 = new TunerConstants2024();
            return tunerConstants2024.DriveTrain;
        } else if (constants.DriveBase == Constants.BotName.Tobor26) {
            tunerConstants2023 = new TunerConstants2023();
            return tunerConstants2023.DriveTrain;
        } else if (constants.DriveBase.equals(Constants.BotName.SIM)) {
            tunerConstantsSIM = new TunerConstantsSIM();
            return tunerConstantsSIM.DriveTrain;
        } else {
            tunerConstantsSIM = new TunerConstantsSIM();
            return tunerConstantsSIM.DriveTrain;
        }
        
    }
}
