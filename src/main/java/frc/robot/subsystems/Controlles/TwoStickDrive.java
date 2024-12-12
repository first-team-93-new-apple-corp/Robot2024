package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoStickDrive implements ControllerSchemeIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;


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
        switch (LeftStick.getHID().getPOV()) {
            case 0:
                return POVs[1];
            case 45:
                return POVs[2];
            case 90:
                return POVs[3];
            case 135:
                return POVs[4];
            case 180:
                return POVs[5];
            case 225:
                return POVs[6];
            case 270:
                return POVs[7];
            case 315:
                return POVs[8];
            default:
                return POVs[0];
        }
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }

    @Override
    public Trigger Brake(){
        return RightStick.trigger();
    }

}
