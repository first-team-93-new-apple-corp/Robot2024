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
    Joystick m_joystick1 = new Joystick(0);
    VisionSubsystem Vision;

    public VisionCommand(SwerveDriveSubsystem drivetrain) {
        Vision = new VisionSubsystem(drivetrain);
    }

    @Override
    public void execute() {
        Vision.hasTargets();
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Middle)) {
            Vision.AutoAimAmp();
        } else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Bottom_Middle)) {
            Vision.AutoAimTrap();
        }
        // Vision.periodic();
    }
    // public void periodic(){
    // Vision.periodic();
    // }

}
