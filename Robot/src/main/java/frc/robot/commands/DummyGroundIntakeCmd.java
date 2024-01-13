
package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.DummyIntakeSubsystem;
import frc.robot.subsystems.OperatorInterfaceSubsystem;

public class DummyGroundIntakeCmd extends CommandBase{
    public DummyIntakeSubsystem m_IntakeSubsystem;
    Joystick js = new Joystick(0);
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public DummyGroundIntakeCmd(DummyIntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        addRequirements();
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