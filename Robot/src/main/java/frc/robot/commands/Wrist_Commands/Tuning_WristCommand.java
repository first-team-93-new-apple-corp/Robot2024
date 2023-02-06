package frc.robot.commands.Wrist_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class Tuning_WristCommand extends CommandBase {


    WristSubsystem m_WristSubsystem; 

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_WristCommand(WristSubsystem m_WristSubsystem) {
        this.m_WristSubsystem = m_WristSubsystem; 
        addRequirements(m_WristSubsystem);
        SmartDashboard.putNumber("Wrist Setpoint In Degrees", 0); 
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        m_WristSubsystem.toSetpoint(SmartDashboard.getNumber("Wrist Setpoint In Degrees", 0));
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
