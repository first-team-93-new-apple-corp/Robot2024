package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class ShooterCommand extends Command {
    XboxController opController = new XboxController(2);
    Joystick driver2 = new Joystick(1);
    ShooterSubsystem m_ShooterSubsystem;
    LEDSubsystem m_LED;
    POVButton up = new POVButton(opController, 0);
    POVButton down = new POVButton(opController, 180);
    public ShooterCommand(ShooterSubsystem m_ShooterSubsystem, LEDSubsystem m_LED) {
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_LED = m_LED;
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
        if (driver2.getRawButton(Constants.Thrustmaster.Trigger) && !opController.getRawButton(Constants.xbox.RightShoulderButton)) { // B
            m_ShooterSubsystem.kicker(1);
            m_LED.shot();
        } else if (driver2.getRawButton(Constants.Thrustmaster.Trigger) && opController.getRawButton(Constants.xbox.RightShoulderButton)) { // B
            m_ShooterSubsystem.ampKicker();
            m_LED.shot();
        } else if (!opController.getRawButton(Constants.xbox.LeftShoulderButton) && !opController.getRawButton(Constants.xbox.X)) {
            m_ShooterSubsystem.kickerStop();
        }

        if (up.getAsBoolean()) {
            m_LED.servoUP();
        } else if (down.getAsBoolean()) {
            m_LED.servoDOWN();
        }
    }
}
