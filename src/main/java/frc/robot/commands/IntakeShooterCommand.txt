package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IntakeShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeShooterCommand extends Command {
    // private static XboxController opController = new XboxController(2);

    IntakeShooterSubsystem m_IntakeShooter;

    public IntakeShooterCommand(IntakeShooterSubsystem m_IntakeShooter) {
        this.m_IntakeShooter = m_IntakeShooter;
    }

    public Command Intake(){
        return m_IntakeShooter.runOnce(() -> m_IntakeShooter.Intake());
    }
    
    public Command kicker(){
        return m_IntakeShooter.runOnce(() -> m_IntakeShooter.kicker(1));
    }

    public Command shoot(){
        return m_IntakeShooter.runOnce(() -> m_IntakeShooter.prime());
    }

    public Command amp(){
        return m_IntakeShooter.runOnce(() -> m_IntakeShooter.shootAmp());
    }
}