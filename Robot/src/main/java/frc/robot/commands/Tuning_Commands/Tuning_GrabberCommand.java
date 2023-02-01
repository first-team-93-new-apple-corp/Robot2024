package frc.robot.commands.Tuning_Commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class Tuning_GrabberCommand extends CommandBase {


    GrabberSubsystem m_GrabberSubsystem; 
    double speed;
    XboxController Operator;

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
        m_GrabberSubsystem.directMotorCommand(speed);//TODO impliment logic
       
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
