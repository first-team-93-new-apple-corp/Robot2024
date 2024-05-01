package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ClimberCommand extends Command {
    private XboxController op;
    private ClimberSubsystem m_ClimberSubsystem;
    private double rightSetpoint = 0;
    private double leftSetpoint = 0;
    private PIDController leftClimberPID = new PIDController(0.05, 0, 0);
    private PIDController rightClimberPID = new PIDController(0.05, 0, 0);

    public ClimberCommand(XboxController op, ClimberSubsystem m_ClimberSubsystem) {
        this.op = op;
        this.m_ClimberSubsystem = m_ClimberSubsystem;
    }

    //--------------------------------------------PREFLIGHT METHODS--------------------------------------------
    public void windLeft(double speed) {
        m_ClimberSubsystem.leftSpeed(speed);
    }

    public void windRight(double speed) {
        m_ClimberSubsystem.rightSpeed(speed);
    }

    public void zeroLeft() {
        m_ClimberSubsystem.zeroLeft();
    }

    public void zeroRight() {
        m_ClimberSubsystem.zeroRight();
    }

    public double leftDraw() {
        return m_ClimberSubsystem.getLeftDraw();
    }

    public double rightDraw() {
        return m_ClimberSubsystem.getRightDraw();
    }
    //--------------------------------------------SETS SETPOINTS--------------------------------------------
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
    //--------------------------------------------SETS SETPOINTS--------------------------------------------
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
    //--------------------------------------------Go To Setpoints--------------------------------------------
    public void calculateLeft() {
        m_ClimberSubsystem.leftSpeed(m_ClimberSubsystem.checkLeftBound(leftClimberPID.calculate(m_ClimberSubsystem.leftPosition(), leftSetpoint)));
    }
    public void calculateRight() {
        m_ClimberSubsystem.rightSpeed(m_ClimberSubsystem.checkRightBound(rightClimberPID.calculate(m_ClimberSubsystem.rightPosition(), rightSetpoint)));
    }
    //--------------------------------------------CLIMBER COMMANDS--------------------------------------------
    private boolean atPID(){
        return leftClimberPID.atSetpoint() && rightClimberPID.atSetpoint();
    }

    public Command driveClimbers() {
        return m_ClimberSubsystem.run(() ->
        {calculateLeft();
        calculateRight();
        }).until(this::atPID);
    }

    public Command hangCommand(){
        return m_ClimberSubsystem.runOnce(this::toHang).andThen(this.driveClimbers());
    }

    public Command stowCommand(){
        return m_ClimberSubsystem.runOnce(this::stow).andThen(this.driveClimbers());
    }   

    public Command climbCommand(){
        return m_ClimberSubsystem.runOnce(this::toClimb).andThen(this.driveClimbers());
    }
    //--------------------------------------------OLD CLIMBER COMMAND D: --------------------------------------------
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
