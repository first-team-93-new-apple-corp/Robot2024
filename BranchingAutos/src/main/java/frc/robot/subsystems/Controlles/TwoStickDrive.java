package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoStickDrive implements ControllerIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;

    public TwoStickDrive(int LeftPort, int RightPort){
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);

    }

    @Override
    public double DriveLeft() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'DriveLeft'");
    }

    @Override
    public double DriveUp() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'DriveUp'");
    }

    @Override
    public Translation2d POV() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'POV'");
    }

    @Override
    public Trigger Seed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Seed'");
    }
    
}
