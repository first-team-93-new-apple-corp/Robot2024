package frc.robot.commands.Tuning_Commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class Tuning_GrabberCommand extends CommandBase {


    GrabberSubsystem m_GrabberSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_GrabberCommand(GrabberSubsystem m_GrabberSubsystem,double speed, XboxController f310) {
        this.m_GrabberSubsystem = m_GrabberSubsystem;
        addRequirements(m_GrabberSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_GrabberSubsystem.toSetpoint();//TODO impliment logic
    }

    @Override
    public void end(boolean interrupted) {
        m_GrabberSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
