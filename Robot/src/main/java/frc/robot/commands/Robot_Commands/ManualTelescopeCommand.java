package frc.robot.commands.Robot_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingSubsystem;

public class ManualTelescopeCommand extends CommandBase {


    TelescopingSubsystem m_TelescopingSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ManualTelescopeCommand(TelescopingSubsystem m_TelescopingSubsystem,double speed) {
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
        m_TelescopingSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
