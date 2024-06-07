package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class XboxDriveExponential extends XboxDriveLinear{
    private final XboxController Xbox;

    /**
    * Creates an object containing the Xbox Controller that will return our input values
    */
    public XboxDriveExponential(int XboxPort){
        super(XboxPort);
        this.Xbox = new XboxController(XboxPort);
    }
    private double SquareWithSign(double value){
        return Math.copySign(value * value, value);
    }
    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            SquareWithSign(-Xbox.getRawAxis(Constants.xbox.Axis.Left_Stick_Y)),
            SquareWithSign(-Xbox.getRawAxis(Constants.xbox.Axis.Left_Stick_X)),
            new Rotation2d(SquareWithSign(-Xbox.getRawAxis(Constants.xbox.Axis.Right_Stick_X))));
    }
}
