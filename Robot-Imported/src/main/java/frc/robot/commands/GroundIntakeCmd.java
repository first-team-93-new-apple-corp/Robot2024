package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IGroundIntake;

public class GroundIntakeCmd extends Command {
    private IGroundIntake m_IntakeSubsystem;
    Joystick js;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public GroundIntakeCmd(IGroundIntake IntakeSubsystem) {
        this.m_IntakeSubsystem = IntakeSubsystem;
        addRequirements(m_IntakeSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (js.getRawButton(3)) { //TODO change with driver input
            m_IntakeSubsystem.intakeStart();
        }else {
            //Stop
           m_IntakeSubsystem.intakeStop(); 
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}