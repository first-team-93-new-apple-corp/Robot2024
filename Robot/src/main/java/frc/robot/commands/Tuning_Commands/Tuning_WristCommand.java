package frc.robot.commands.Tuning_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class Tuning_WristCommand extends CommandBase {


    WristSubsystem m_WristSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_WristCommand(WristSubsystem m_WristSubsystem,double speed, XboxController f310) {
        this.m_WristSubsystem = m_WristSubsystem; 
        this.speed = speed;
        addRequirements(m_WristSubsystem);

    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        m_WristSubsystem.toSetpoint(speed);//TODO impliment logic
    }

    @Override
    public void end(boolean interrupted) {
        m_WristSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
