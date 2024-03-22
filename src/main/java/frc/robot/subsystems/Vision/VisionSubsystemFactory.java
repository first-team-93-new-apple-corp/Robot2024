package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Vision.IO.VisionIOPhotonReal;
import frc.robot.subsystems.Vision.IO.VisionIOReal;
import frc.robot.subsystems.Vision.IO.VisionIOSim;

public class VisionSubsystemFactory {
    public static VisionSubsystem build(SwerveDriveSubsystem m_driveSubsystem, VisionConstants constants) {
        if (Utils.isSimulation()) {
            if (constants.SimEnabled){
                return new VisionSubsystem(new VisionIOPhotonReal(constants), m_driveSubsystem);
            } else {
                // Real Vision will cause null data meaning its never used
                return new VisionSubsystem(new VisionIOReal(constants), m_driveSubsystem);
            }
        } else {
            return new VisionSubsystem(new VisionIOReal(constants), m_driveSubsystem);
        }        
    }
}
