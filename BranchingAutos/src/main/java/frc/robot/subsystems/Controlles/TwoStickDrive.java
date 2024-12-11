package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoStickDrive implements ControllerIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;
    private double POVDistance = .45;
    private double POVDistanceDiagonal = Math.sqrt(2 * (Math.pow(POVDistance, 2)));
    private Translation2d[] POVs = {
            new Translation2d(0, 0), // Default
            new Translation2d(0, POVDistance), // left 1
            new Translation2d(POVDistance, 0), // Up 2
            new Translation2d(0, -POVDistance), // Right 3
            new Translation2d(-POVDistance, 0), // Down 4
            new Translation2d(POVDistanceDiagonal, POVDistanceDiagonal), // up left 5
            new Translation2d(POVDistanceDiagonal, -POVDistanceDiagonal), // up right 6
            new Translation2d(-POVDistanceDiagonal, POVDistanceDiagonal), // down left 7
            new Translation2d(-POVDistanceDiagonal, -POVDistanceDiagonal) // down right 8
    };

    public TwoStickDrive(int LeftPort, int RightPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);

    }

    @Override
    public double InputLeft() {
        return -LeftStick.getY();
    }

    @Override
    public double InputUp() {
        return -LeftStick.getX();
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX();
    }

    @Override
    public Translation2d POV() {
        if (LeftStick.povLeft().getAsBoolean()) {
            return POVs[1];
        } else if (LeftStick.povUp().getAsBoolean()) {
            return POVs[2];
        } else if (LeftStick.povRight().getAsBoolean()) {
            return POVs[3];
        } else if (LeftStick.povDown().getAsBoolean()) {
            return POVs[4];
        } else if (LeftStick.povUpLeft().getAsBoolean()) {
            return POVs[5];
        } else if (LeftStick.povUpRight().getAsBoolean()) {
            return POVs[6];
        } else if (LeftStick.povDownLeft().getAsBoolean()) {
            return POVs[7];
        } else if (LeftStick.povDownRight().getAsBoolean()) {
            return POVs[8];
        } else {
            return POVs[0];
        }
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }

}
