package frc.robot.commands.Robot_Commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class Manual_GrabberCommand extends CommandBase {


    GrabberSubsystem m_GrabberSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Manual_GrabberCommand(GrabberSubsystem m_GrabberSubsystem,double speed) {
        this.m_GrabberSubsystem = m_GrabberSubsystem; 
        this.speed = speed;
        addRequirements(m_GrabberSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_GrabberSubsystem.directMotorCommand(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_GrabberSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
