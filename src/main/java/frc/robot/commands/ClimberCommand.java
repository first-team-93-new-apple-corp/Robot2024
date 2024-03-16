package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ClimberCommand extends Command {
    XboxController op;
    ClimberSubsystem m_climber;
    double rightSetpoint = 0;
    double leftSetpoint = 0;
    PIDController leftClimberPID = new PIDController(0.05, 0, 0);
    PIDController rightClimberPID = new PIDController(0.05, 0, 0);

    public ClimberCommand(XboxController op, ClimberSubsystem m_ClimberSubsystem) {
        this.op = op;
        m_climber = m_ClimberSubsystem;
    }

    // public void leftToSetpoint(double setpoint) {
    //     m_climber.leftSpeed(leftPID.calculate(m_climber.leftPosition(), leftSetpoint));
    // }
    // public void rightToSetpoint(double setpoint) {
    //     m_climber.rightSpeed(rightPID.calculate(m_climber.rightPosition(), rightSetpoint));
    // }

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

    public void leftSetpoint(double setpoint) {
        leftSetpoint = setpoint;
    }
    public void rightSetpoint(double setpoint) {
        rightSetpoint = setpoint;
    }

    public void changeLeft(double amount) {
        leftSetpoint += amount;
    }
    
    public void changeRight(double amount) {
        rightSetpoint += amount;
    }
    public void toHang() {
        leftSetpoint(-150);
        rightSetpoint(-150);
    }
    public void stow() {
        leftSetpoint(-1);
        rightSetpoint(-1);
    }
    public void toClimb() {
        leftSetpoint(-15);
        rightSetpoint(-15);
    }
    public void calculateLeft() {
        m_climber.leftSpeed(m_climber.checkLeftBound(leftClimberPID.calculate(m_climber.leftPosition(), leftSetpoint)));
    }
    public void calculateRight() {
        m_climber.rightSpeed(m_climber.checkRightBound(rightClimberPID.calculate(m_climber.rightPosition(), rightSetpoint)));
    }
    @Override
    public void execute() {
        if(op.getRawButtonPressed(Constants.xbox.A)) {
            stow();
        } else if(op.getRawButtonPressed(Constants.xbox.B)) {
            toHang();
        }
        calculateLeft();
        calculateRight();
        SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
        SmartDashboard.putNumber("Right Setpoint", rightSetpoint);
        // testClimbers();
    }
}
