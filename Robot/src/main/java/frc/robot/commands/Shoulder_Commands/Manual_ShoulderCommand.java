package frc.robot.commands.Shoulder_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;

public class Manual_ShoulderCommand extends CommandBase {

    ShoulderSubsystem m_ShoulderSubsystem;
    double speed;
    XboxController controller; 

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public 
    Manual_ShoulderCommand(ShoulderSubsystem m_ShoulderSubsystem, double speed, XboxController controller) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
        this.speed = speed;
        this.controller = controller;

        addRequirements(m_ShoulderSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_ShoulderSubsystem.directMotorCommand(Constants.checkJoystickDeadband(-controller.getRawAxis(1), 0.04) * speed);
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
