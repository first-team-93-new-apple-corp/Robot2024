package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class TwoStickExponentialDrive implements InputsIO{
    private final Joystick Joystick1;
    private final Joystick Joystick2;

    /**
    * Creates an object containing the sticks that will return our input values
    */
    public TwoStickExponentialDrive(int Joystick1Port, int Joystick2Port){
        this.Joystick1 = new Joystick(Joystick1Port);
        this.Joystick2 = new Joystick(Joystick2Port);
    }
    private double SquareWithSign(double value){
        return Math.copySign(value * value, value);
    }
    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            SquareWithSign(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)),
            SquareWithSign(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)),
            new Rotation2d(SquareWithSign(-Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x))));
    }
}
