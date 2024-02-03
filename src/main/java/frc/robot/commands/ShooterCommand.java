package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
    XboxController opController = new XboxController(2);
    Joystick driver2 = new Joystick(1);
    @Override
    public void execute() {
        ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        // Stuff for the shooter
        if (opController.getRawButton(Constants.F310_D.RightTrigger)) { // RightTrigger
            m_ShooterSubsystem.prime();
        } else if (opController.getRawButton(Constants.F310_D.RightShoulderButton)) { // RightShoulderButton
            m_ShooterSubsystem.shootAmp();
        } else if (opController.getRawButton(Constants.F310_D.LeftShoulderButton)) { // LeftShoulderButton
            m_ShooterSubsystem.intakeFront();
        } else {
            m_ShooterSubsystem.shooterStop();
        }


        // For the Kicker
        if (driver2.getRawButton(Constants.Thrustmaster.Trigger)) { // B
            m_ShooterSubsystem.kicker();
        } else {
            m_ShooterSubsystem.kickerStop();
        }
    }
}
