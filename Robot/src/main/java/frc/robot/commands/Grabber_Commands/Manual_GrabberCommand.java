package frc.robot.commands.Grabber_Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class Manual_GrabberCommand extends CommandBase {


    GrabberSubsystem m_GrabberSubsystem; 
    double speed;
    XboxController controller;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Manual_GrabberCommand(GrabberSubsystem m_GrabberSubsystem,double speed, XboxController controller) {
        this.m_GrabberSubsystem = m_GrabberSubsystem; 
        this.speed = speed;
        this.controller = controller;
        addRequirements(m_GrabberSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_GrabberSubsystem.directMotorCommand(Constants.checkJoystickDeadzone(-controller.getRawAxis(0), 0.04) * speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_GrabberSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
