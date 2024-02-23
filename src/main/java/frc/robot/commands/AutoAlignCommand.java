
package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants;
import frc.robot.subsystems.AutoAlignSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class AutoAlignCommand extends Command {
    Joystick m_joystick1 = new Joystick(0);
    XboxController m_xboxController = new XboxController(2);

    AutoAlignSubsystem m_AutoAlignSubsystem;


    public AutoAlignCommand(SwerveDriveSubsystem drivetrain) {
        m_AutoAlignSubsystem = new AutoAlignSubsystem(drivetrain);
    }


    @Override
    public void execute() {
        m_AutoAlignSubsystem.periodic();
        if (m_xboxController.getRawButton(6)) {
            m_AutoAlignSubsystem.AutoAimAmp();
        } else if(m_xboxController.getRawButton(5)){
            m_AutoAlignSubsystem.AutoAimTrap();
        }
    }
}
