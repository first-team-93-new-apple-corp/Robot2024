package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private static XboxController opController = new XboxController(2);
    IntakeSubsystem m_IntakeSubsystem;
    LEDSubsystem m_LED;
    public int IntakeSpeed;

    public IntakeCommand(ShooterSubsystem m_shooter, IntakeSubsystem m_IntakeSubsystem, LEDSubsystem m_LED) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.m_LED = m_LED;
    }

    public Command AutoIntake() {
        return m_IntakeSubsystem.runOnce(() -> m_IntakeSubsystem.intake());
    }

    public Command AutonStopIntake() {
        return m_IntakeSubsystem.runOnce(() -> m_IntakeSubsystem.stop());
    }
    public Command Intake(){
        return m_IntakeSubsystem.run((m_IntakeSubsystem::intake));
    }

    public Command StopIntake(){
        return m_IntakeSubsystem.runOnce((() -> {m_IntakeSubsystem.stop();
            m_IntakeSubsystem.resetIntakeState();
            opController.setRumble(RumbleType.kBothRumble, 0);
        }));
    }

    @Override
    public void execute() {
        // For the Intake
        if (opController.getRawButton(Constants.xbox.X)) { // X
            m_IntakeSubsystem.intake();
        } else if (opController.getRawButton(Constants.xbox.A)) { // A
            // m_IntakeSubsystem.passthrough();
        } else {
            m_IntakeSubsystem.stop();
            m_IntakeSubsystem.resetIntakeState();
            opController.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
