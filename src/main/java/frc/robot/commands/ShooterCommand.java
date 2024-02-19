package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
    XboxController opController = new XboxController(2);
    Joystick driver2 = new Joystick(1);
    ShooterSubsystem m_ShooterSubsystem;

    public ShooterCommand() {
        m_ShooterSubsystem = new ShooterSubsystem();
    }

    @Override
    public void execute() {
        // Stuff for the shooter
        if (opController.getRawAxis(Constants.xbox.Axis.RT) > 0.6) { // RightTrigger
            m_ShooterSubsystem.prime();
        } else if (opController.getRawButton(Constants.xbox.RightShoulderButton)) { // RightShoulderButton
            m_ShooterSubsystem.shootAmp();
        } else if (opController.getRawButton(Constants.xbox.LeftShoulderButton)) { // LeftShoulderButton
            m_ShooterSubsystem.intakeFront();
        } else if (!opController.getRawButton(Constants.xbox.X)){
            m_ShooterSubsystem.shooterStop();
        }

        if (opController.getRawButtonPressed(Constants.xbox.Menu)) {
            m_ShooterSubsystem.increaseSpeed();
        } else if (opController.getRawButtonPressed(Constants.xbox.Window)) {
            m_ShooterSubsystem.decreaseSpeed();
        }

        // For the Kicker
        if (driver2.getRawButton(Constants.Thrustmaster.Trigger)) { // B
            m_ShooterSubsystem.kicker(1);
        } else if (!opController.getRawButton(Constants.xbox.LeftShoulderButton) && !opController.getRawButton(Constants.xbox.X)) {
            m_ShooterSubsystem.kickerStop();
        }
    }
}
