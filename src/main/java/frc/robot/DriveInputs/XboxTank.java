package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

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
    * Returns a Field Relative ChassisSpeeds
    */
    @Override
    public ChassisSpeeds fieldSpeeds(Rotation2d RobotAngle){
    return inputSpeeds();
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
