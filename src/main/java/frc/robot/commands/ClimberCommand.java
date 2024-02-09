package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    XboxController op;
    ClimberSubsystem m_climber;
    double rightSetpoint = 0;
    double leftSetpoint = 0;
    PIDController leftPID = new PIDController(0.05, 0, 0);
    PIDController rightPID = new PIDController(0.05, 0, 0);

    public ClimberCommand(XboxController op) {
        this.op = op;
        m_climber = new ClimberSubsystem(op);
    }

    public ClimberCommand() {
        m_climber = new ClimberSubsystem(op);
    }

    public void leftToSetpoint(double setpoint) {
        m_climber.leftSpeed(leftPID.calculate(m_climber.leftPosition(), leftSetpoint));
    }
    public void rightToSetpoint(double setpoint) {
        m_climber.rightSpeed(rightPID.calculate(m_climber.rightPosition(), rightSetpoint));
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
    public void changeLeft(double amount) {
        leftSetpoint += amount;
    }
    
    public void changeRight(double amount) {
        rightSetpoint += amount;
    }
    @Override
    public void execute() {
        // testClimbers();
        //TODO: add a binding for changing the setpoints that would also allow for manual control.
        //TODO: add a binding for zeroing the encoders
    }
}
