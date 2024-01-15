
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Interfaces.IShooter;

public class ShooterCmd extends Command {
    private IShooter m_ShooterSubsystem;
    private Joystick js;

    public ShooterCmd(IShooter ShooterSubsystem) {
        this.m_ShooterSubsystem = ShooterSubsystem;
        addRequirements(m_ShooterSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
        // TODO review later with rest of hardware design.
    }

    @Override
    public void execute() {
        if (js.getRawButton(1)) { // TODO button number will change later with driver input
            // Shoot Amp
            m_ShooterSubsystem.shootAmp();
        } else if (js.getRawButton(2)) { // TODO button number will change later with driver input
            // Shoot Speaker
            m_ShooterSubsystem.shootSpeaker();
        } else {
            // Stop
            m_ShooterSubsystem.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
