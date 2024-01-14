
package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IGroundIntake;
import frc.robot.subsystems.OperatorInterfaceSubsystem;

public class GroundIntakeCmd extends CommandBase{
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
       if (js.getRawButtonPressed(1)) { // Change port
            m_IntakeSubsystem.intakeStart();
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