package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class OneStickLinear implements InputsIO{
    private final Joystick Joystick1;

    /**
    * Creates an object containing the sticks that will return our input values
    */
    public OneStickLinear(int Joystick1Port){
        this.Joystick1 = new Joystick(Joystick1Port);
    }

    @Override
    public Pose2d Inputs() {
        return new Pose2d(
            -Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y),
            -Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x),
            new Rotation2d(-Joystick1.getRawAxis(Constants.Thrustmaster.Axis.rotate)));
    }

    @Override
    public Trigger brake() {
        return new Trigger(Joystick1.button(Constants.Thrustmaster.Trigger, loop));
    }
    @Override
    public Trigger fieldRelButton() {
        return new Trigger(Joystick1.button(Constants.Thrustmaster.Left_Buttons.Top_Middle, loop));
    }
    @Override
    public Trigger robotRelButtonke() {
        return new Trigger(Joystick1.button(Constants.Thrustmaster.Left_Buttons.Bottom_Middle, loop));
    }
    @Override
    public Trigger ampAlignButton() {
        return new Trigger(Joystick1.button(Constants.Thrustmaster.Center_Button, loop));
    }
}
