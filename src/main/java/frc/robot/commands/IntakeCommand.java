package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Code24.IntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private static XboxController opController = new XboxController(2);
    IntakeSubsystem m_IntakeSubsystem;
    public TalonFX frontIntake = new TalonFX(Constants.CTRE.RIO.F_Intake, "rio"),
                   backIntake = new TalonFX(Constants.CTRE.RIO.B_Intake, "rio");
    public int IntakeSpeed;

    public IntakeCommand() {
        m_IntakeSubsystem = new IntakeSubsystem();
    }

   

    @Override
    public void execute() {
        // For the Intake
        if (opController.getRawButton(Constants.xbox.X)) { // X
            m_IntakeSubsystem.Intake();
        } else if (opController.getRawButton(Constants.xbox.A)) { // A
            m_IntakeSubsystem.passthrough();
        } else {
            m_IntakeSubsystem.stop();
        }
    }
}