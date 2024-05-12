package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class XboxDriveLinear implements InputsIO{
    private final XboxController Xbox;

    /**
    * Creates an object containing the Xbox Controller that will return our input values
    */
    public XboxDriveLinear(int XboxPort){
        this.Xbox = new XboxController(XboxPort);
    }

    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            -Xbox.getRawAxis(Constants.xbox.Axis.Left_Stick_Y),
            -Xbox.getRawAxis(Constants.xbox.Axis.Left_Stick_X),
            new Rotation2d(-Xbox.getRawAxis(Constants.xbox.Axis.Right_Stick_X)));
    }
    
}
