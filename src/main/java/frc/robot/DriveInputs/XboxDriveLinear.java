package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    @Override
    public Trigger brake() {
        return new Trigger(loop, () -> (Xbox.getRawAxis(2) > .8));
    }
    @Override
    public Trigger fieldRelButton() {
        return new Trigger(loop, Xbox::getXButton);
        
    }
    @Override
    public Trigger robotRelButtonke() {
        return new Trigger(loop, () -> (Xbox.getRawAxis(3) > .8));
        
    }
    @Override
    public Trigger ampAlignButton() {
        return new Trigger(loop, Xbox::getYButton);
    }

}
