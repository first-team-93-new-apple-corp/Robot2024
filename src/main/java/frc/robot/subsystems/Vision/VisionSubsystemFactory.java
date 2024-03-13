package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants;
import frc.robot.subsystems.Vision.IO.VisionIOReal;
import frc.robot.subsystems.Vision.IO.VisionIOSim;

public class VisionSubsystemFactory {
    public static VisionSubsystem build() {
        if (Utils.isSimulation()) {
            return new VisionSubsystem(new VisionIOSim());
        } else {
            return new VisionSubsystem(new VisionIOReal());
        }        
    }
}
