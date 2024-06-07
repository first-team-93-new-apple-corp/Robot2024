package frc.robot.DriveInputs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class XboxAcellerationDrive extends XboxDriveLinear{

    public XboxAcellerationDrive(int XboxPort) {
        super(XboxPort);
    }

    private ChassisSpeeds oldChassisSpeeds = new ChassisSpeeds();

    /**
    * Creates an object containing the sticks that will return our input values
    */

    @Override
    public ChassisSpeeds inputSpeeds(){
        oldChassisSpeeds = oldChassisSpeeds.plus(new ChassisSpeeds(
            (checkDeadzone(ForwardMetersPerSecond())),
            (checkDeadzone(SidewaysMetersPerSecond())),
            (checkDeadzone(omegaRadiansPerSecond()))).times(.02));
        return oldChassisSpeeds;
    }    
}
