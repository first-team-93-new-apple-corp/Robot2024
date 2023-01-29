package frc.robot.commands.Tuning_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingSubsystem;

public class Tuning_TelescopeCommand extends CommandBase {


    TelescopingSubsystem m_TelescopingSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_TelescopeCommand(TelescopingSubsystem m_TelescopingSubsystem,double speed, XboxController f310) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;
        this.speed = speed;
        addRequirements(m_TelescopingSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_TelescopingSubsystem.toSetpoint(speed);//TODO impliment logic
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
