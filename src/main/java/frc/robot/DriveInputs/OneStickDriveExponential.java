package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class OneStickDriveExponential extends OneStickLinear{
    private final Joystick Joystick1;
    /**
    * Creates an object containing the sticks that will return our input values
    */
    public OneStickDriveExponential(int Joystick1Port){
        super(Joystick1Port);
        this.Joystick1 = new Joystick(Joystick1Port);
    }
    private double SquareWithSign(double value){
        return Math.copySign(value * value, value);
    }
    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            SquareWithSign(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)),
            SquareWithSign(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)),
            new Rotation2d(SquareWithSign(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.rotate))));
    }
}
