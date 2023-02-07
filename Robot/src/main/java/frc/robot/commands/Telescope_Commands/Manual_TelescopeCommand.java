package frc.robot.commands.Telescope_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingSubsystem;

public class Manual_TelescopeCommand extends CommandBase {


    TelescopingSubsystem m_TelescopingSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Manual_TelescopeCommand(TelescopingSubsystem m_TelescopingSubsystem,double speed) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem; 
        this.speed = speed;
        addRequirements(m_TelescopingSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_TelescopingSubsystem.directMotorCommand(speed);
    
    }

    @Override
    public void end(boolean interrupted) {
        m_TelescopingSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
