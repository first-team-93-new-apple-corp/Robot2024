
package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AutoAlignSubsystem;


public class AutoAlignCommand extends Command {
    Joystick m_joystick1 = new Joystick(0);


    AutoAlignSubsystem m_AutoAlignSubsystem;


    public AutoAlignCommand(SwerveDriveSubsystem drivetrain) {
        m_AutoAlignSubsystem = new AutoAlignSubsystem(drivetrain);
    }


    @Override
    public void execute() {
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
            m_AutoAlignSubsystem.AutoAimAmp();
        } else if(m_joystick1.getRawButton(Constants.Thrustmaster.Right_Button)){
            m_AutoAlignSubsystem.AutoAimTrap();
        }
    }
}
