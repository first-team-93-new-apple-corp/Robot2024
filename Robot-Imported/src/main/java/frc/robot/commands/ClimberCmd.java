package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IClimber;

public class ClimberCmd extends Command {
    private IClimber m_climberSubsystem;
    Joystick js = new Joystick(0);
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ClimberCmd(IClimber ClimberSubsystem) {
        this.m_climberSubsystem = ClimberSubsystem;
        addRequirements(m_climberSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       if (js.getRawButtonPressed(4)) { //TODO change with driver imput
            m_climberSubsystem.raise();
       } else if (js.getRawButtonPressed(5)){ //TODO change with driver imput
            m_climberSubsystem.lower();
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
