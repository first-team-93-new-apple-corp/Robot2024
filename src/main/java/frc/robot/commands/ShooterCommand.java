package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommand extends Command {
    private XboxController opController = new XboxController(2);
    private Joystick driver2 = new Joystick(1);
    private ShooterSubsystem m_ShooterSubsystem;
    private LEDSubsystem m_LED;
    public ShooterCommand(ShooterSubsystem m_ShooterSubsystem, LEDSubsystem m_LED) {
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_LED = m_LED;
    }
    public Command AutonAmp() {
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.AmpForAuton());
    }

    public Command AutonShooter() {
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.prime()).andThen(Commands.waitSeconds(1)).andThen(m_ShooterSubsystem.runOnce(()-> m_ShooterSubsystem.kicker(1)));
    }

    public Command AutonStopShooter() {
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.shooterStop());
    }

    public Command AutonDribbleNote(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.DribbleOutNote());
    }

    public Command Prime(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.prime());
    }

    public Command ShootAmp(){
        return m_ShooterSubsystem.runOnce(() -> {m_ShooterSubsystem.shootAmp();
        });
    }

    public Command IntakeFront() {
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.intakeFront());
    }

    public Command StopShooter() {
        return m_ShooterSubsystem.runOnce(() -> {
            m_ShooterSubsystem.shooterStop();
        });
    }

    public Command Kicker(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.kicker(1));
    }

    public Command AmpKicker(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.ampKicker());
    }

    public Command KickerStop(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.kickerStop());
    }
    
    public Command increseSpeed(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.increaseSpeed());
    }
    public Command decreaseSpeed(){
        return m_ShooterSubsystem.runOnce(() -> m_ShooterSubsystem.decreaseSpeed());
    }

    @Override
    public void execute() {
        // Stuff for the shooter
        if (opController.getRawAxis(Constants.xbox.Axis.RT) > 0.6) { // RightTrigger
            m_ShooterSubsystem.prime();
            m_LED.LEDSHOOT().schedule();
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
            // m_LED.LEDSHOOT().schedule();

        } else if (driver2.getRawButton(Constants.Thrustmaster.Trigger) && opController.getRawButton(Constants.xbox.RightShoulderButton)) { // B
            m_ShooterSubsystem.ampKicker();
            // m_LED.LEDSHOOT().schedule();
        } else if (!opController.getRawButton(Constants.xbox.LeftShoulderButton) && !opController.getRawButton(Constants.xbox.X)) {
            m_ShooterSubsystem.kickerStop();
            // m_LED.turnLEDSOff();
        }
    }
}
