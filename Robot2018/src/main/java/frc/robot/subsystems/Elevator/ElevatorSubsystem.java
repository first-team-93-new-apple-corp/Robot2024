package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ElevatorSubsystem implements Subsystem {
    private TalonSRX m_ElevatorMotor;
    private double upSpeed = -0.5;
    private double downSpeed = -0.05;
    public ElevatorSubsystem() {
        //TODO: Quadrature Encoder?
        m_ElevatorMotor = new TalonSRX(6);

        m_ElevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void moveUp() {
        m_ElevatorMotor.set(TalonSRXControlMode.PercentOutput, upSpeed);
    }
    public void moveDown() {
        m_ElevatorMotor.set(TalonSRXControlMode.PercentOutput, downSpeed);
    }
    public void setSpeed(double speed) {
        m_ElevatorMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
    public void stop() {
        m_ElevatorMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
