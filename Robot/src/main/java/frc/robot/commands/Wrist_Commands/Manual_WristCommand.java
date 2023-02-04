package frc.robot.commands.Wrist_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class Manual_WristCommand extends CommandBase {


    WristSubsystem m_WristSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Manual_WristCommand(WristSubsystem m_WristSubsystem,double speed) {
        this.m_WristSubsystem = m_WristSubsystem; 
        this.speed = speed;
        addRequirements(m_WristSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_WristSubsystem.directMotorCommand(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_WristSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
