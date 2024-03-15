package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Vision.IO.VisionIOReal;
import frc.robot.subsystems.Vision.IO.VisionIOSim;

public class VisionSubsystemFactory {
    public static VisionSubsystem build(SwerveDriveSubsystem m_driveSubsystem) {
        if (Utils.isSimulation()) {
            if (Constants.VisionConstants.SimEnabled){
                return new VisionSubsystem(new VisionIOSim(), m_driveSubsystem);
            } else {
                // Real Vision will cause null data meaning its never used
                return new VisionSubsystem(new VisionIOReal(), m_driveSubsystem);
            }
        } else {
            return new VisionSubsystem(new VisionIOReal(), m_driveSubsystem);
        }        
    }
}
