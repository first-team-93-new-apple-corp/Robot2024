
package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.DummyClimberSubsystem;
import frc.robot.subsystems.OperatorInterfaceSubsystem;

public class DummyClimberCommand extends CommandBase {
    public DummyClimberSubsystem m_climberSubsystem;
    Joystick js = new Joystick(0);
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public DummyClimberCommand(DummyClimberSubsystem m_ClimberSubsystem) {
        this.m_climberSubsystem = m_ClimberSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       if (js.getRawButtonPressed(1)) { // Change port
            m_climberSubsystem.raise();
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
