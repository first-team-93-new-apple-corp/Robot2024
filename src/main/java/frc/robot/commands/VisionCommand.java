package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
    Joystick m_joystick1 = new Joystick(0);

    VisionSubsystem Vision;

    public VisionCommand(SwerveDriveSubsystem drivetrain) {
        Vision = new VisionSubsystem(drivetrain);
    }

    @Override
    public void execute() {
        Vision.hasTargets();
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Middle)) {
            Vision.AutoAim();
        } else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Left)) {
            Vision.LimeLightOn();
        } else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Bottom_Left)) {
            Vision.LimeLightOff();
        }
    }

}
