package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    XboxController op;
    ClimberSubsystem m_climber;
    public ClimberCommand(XboxController op) {
        this.op = op;
        m_climber = new ClimberSubsystem(op);
    }
    public ClimberCommand() {
        m_climber = new ClimberSubsystem(op);
    }

    public void testClimbers() {
            m_climber.leftSpeed(op.getRawAxis(Constants.xbox.Axis.Left_Stick_Y));
            m_climber.rightSpeed(op.getRawAxis(Constants.xbox.Axis.Right_Stick_Y));

    }
    public void windLeft(double speed) {
        m_climber.leftSpeed(speed);
    }
    public void windRight(double speed) {
        m_climber.rightSpeed(speed);
    }
    public void zeroLeft() {
        m_climber.zeroLeft();
    }
    public void zeroRight() {
        m_climber.zeroRight();
    }
    public double leftDraw() {
        return m_climber.getLeftDraw();
    }
    public double rightDraw() {
        return m_climber.getRightDraw();
    }
    @Override
    public void execute() {
        testClimbers();
    }
}
