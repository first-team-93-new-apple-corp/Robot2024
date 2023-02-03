package frc.robot.commands.Tuning_Commands;

import org.ejml.data.FGrowArray;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class Tuning_GrabberCommand extends CommandBase {

    GrabberSubsystem m_GrabberSubsystem;
    double speed;
    XboxController f310;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_GrabberCommand(GrabberSubsystem m_GrabberSubsystem, double speed, XboxController f310) {
        this.m_GrabberSubsystem = m_GrabberSubsystem;
        this.speed = speed;
        this.f310 = f310;
        addRequirements(m_GrabberSubsystem);

        SmartDashboard.putNumber("Grabber Speed", 0);

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
