package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.IO.ClimberIO;
import frc.robot.subsystems.Climber.IO.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    
    private final ClimberIO m_io;
    private ClimberIOInputs m_Inputs = new ClimberIOInputs();

    public ClimberSubsystem(ClimberIO io) {
        m_io = io;
    }
    
    public void leftSpeed(double speed) {
        m_io.leftSpeed(speed);
    }
    public void rightSpeed(double speed) {
        m_io.rightSpeed(speed);
    }
    public void zeroLeft() {
        m_io.zeroLeft();
    }
    public void zeroRight() {
        m_io.zeroRight();
    }
    public double getLeftDraw() {
        return m_io.getLeftDraw();
    }
    public double getRightDraw() {
        return m_io.getRightDraw();
    }
    public double leftPosition() {
        return m_io.leftPosition();
    }
    public double rightPosition() {
        return m_io.rightPosition();
    }

    public double checkLeftBound(double output) {
        return m_io.checkLeftBound(output);
    }
    public double checkRightBound(double output) {
        return m_io.checkRightBound(output);
    }
    @Override
    public void periodic() {
        m_io.updateValues(m_Inputs);
        SignalLogger.writeDouble("Climber:Left Position", leftPosition());
        SignalLogger.writeDouble("Climber:Right Position", rightPosition());

        SmartDashboard.putNumber("L_POS", m_io.leftPosition());
        SmartDashboard.putNumber("R_POS", m_io.getRightDraw());
    }
}
