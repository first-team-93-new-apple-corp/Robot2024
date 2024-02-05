package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class IntakeCommand extends Command {
    XboxController opController = new XboxController(2);

    IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    NoteDetectionSubsystem m_NoteDetect = new NoteDetectionSubsystem();
    ShooterSubsystem m_Shooter = new ShooterSubsystem();

    @Override
    public void execute() {
        // For the Intake
        if (opController.getRawButton(Constants.F310_D.X)) { // X
            m_IntakeSubsystem.Intake();
        } else if (opController.getRawButton(Constants.F310_D.A)) { // A
            m_IntakeSubsystem.passthrough();
        } else if (opController.getRawButton(0)){ // Button for manual stop
            m_IntakeSubsystem.stop();
        } else {
            // Note Detection
            if (!m_NoteDetect.ifAboveKicker() && !m_NoteDetect.ifBelowKicker()) {
                System.out.println("Sensor Intaking");
                m_IntakeSubsystem.Intake();
            } else  if (m_NoteDetect.ifBelowKicker() && !m_NoteDetect.ifAboveKicker()) {
                m_Shooter.kicker();
            } else {
                m_IntakeSubsystem.passthrough();
            }
        }
    }
}