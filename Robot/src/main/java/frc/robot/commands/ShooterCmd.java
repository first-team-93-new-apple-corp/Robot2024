
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Interfaces.IShooter;
import frc.robot.Constants;

public class ShooterCmd extends CommandBase {
    private IShooter m_ShooterSubsystem;
    private Joystick js;
    private XboxController xjs;
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
        if (js.getRawButton(1) || xjs.getRawButton(Constants.F310.RightShoulderButton)) { // TODO button number will change later with driver input
            // Shoot Amp
            m_ShooterSubsystem.shootAmp();
        } else if (js.getRawButton(2) || xjs.getRawButton(Constants.F310.Start)) { // TODO button number will change later with driver input
            // Shoot Speaker
            m_ShooterSubsystem.shootSpeaker();
        } else if (js.getRawButton(3) || xjs.getRawButton(Constants.F310.B)){ // TODO button number will change later with driver input
            //Move note into shooter (transport to shooter motors
            m_ShooterSubsystem.intake();
        } else if (js.getRawButton(4) || xjs.getRawButton(Constants.F310.X)){ // TODO button number will change later with driver input
            //muzle intake into shooter from sorce
            m_ShooterSubsystem.muzzleIntake();
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
