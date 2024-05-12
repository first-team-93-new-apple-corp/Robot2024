package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class XboxTank implements InputsIO{
    private final XboxController Xbox;

    /**
    * Creates an object containing the Xbox Controller that will return our input values
    */
    public XboxTank(int XboxPort){
        this.Xbox = new XboxController(XboxPort);
    }
    public double LeftInput(){
        return -Xbox.getRawAxis(Constants.xbox.Axis.Left_Stick_Y);
    }

    public double RightInput(){
        return -Xbox.getRawAxis(Constants.xbox.Axis.Right_Stick_Y);
    }

    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            (LeftInput() + RightInput())/2,
            0,
            new Rotation2d((RightInput()-LeftInput())/2));
    }

    /**
    * Returns a Field Relitive ChassisSpeeds
    */
    @Override
    public ChassisSpeeds fieldSpeeds(SwerveDriveSubsystem m_SwerveDriveSubsystem){
    return inputSpeeds();
    } 
    
}
