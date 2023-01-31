package frc.robot.commands.Tuning_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class Tuning_ShoulderCommand extends CommandBase {


    ShoulderSubsystem m_ShoulderSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_ShoulderCommand(ShoulderSubsystem m_ShoulderSubsystem, XboxController f310) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem; 
        SmartDashboard.putNumber("Shoulder Setpoint", 0);

        addRequirements(m_ShoulderSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_ShoulderSubsystem.toSetpoint(SmartDashboard.getNumber("Shoulder Setpoint", 0));
    }

    @Override
    public void end(boolean interrupted) {
        m_ShoulderSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
