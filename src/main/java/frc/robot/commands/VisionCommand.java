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
        Vision.periodic();
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
            Vision.AlignAmp();
        } else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Button)) {
            Vision.AlignTrap();
        } else {
            Vision.resetStateTrap();
            Vision.resetStateAmp();
        }
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Left)) {
            Vision.LimeLightOn();
        } else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Buttons.Bottom_Left)) {
            Vision.LimeLightOff();
        }
    }

}
