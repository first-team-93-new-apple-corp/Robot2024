package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
    // double Ttx = -3.6665;
    // double Tty = 20.84;
    // double Tta = 2.61;
    // double Ttl = 22-25;
    Joystick joystick1 = new Joystick(1);
    VisionSubsystem Vision;

    public VisionCommand() {
        Vision = new VisionSubsystem();
    }

    @Override
    public void execute() {
        Vision.hasTargets();
        if (joystick1.getRawButton(Constants.Thrustmaster.Right_Button)) {
            Vision.AutoAimAmp();
        } else if (joystick1.getRawButton(Constants.Thrustmaster.Left_Button)) {
            Vision.AutoAimTrap();
        }
        // Vision.periodic();
    }
    // public void periodic(){
    // Vision.periodic();
    // }

}
