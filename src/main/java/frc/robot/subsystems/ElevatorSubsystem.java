package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    DigitalInput topLimit;
    DigitalInput bottomLimit;
    TalonFX m_motor;
    TalonFXConfiguration m_motorconfig;
    PositionVoltage m_control = new PositionVoltage(0);

    public ElevatorSubsystem() {
        m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motorconfig.Slot0.kG = 0.36;
        m_motorconfig.Slot0.kV = 2.38;
        m_motorconfig.Slot0.kA = 0.05;
        m_motorconfig.Slot0.kP = 0.05;
        m_motorconfig.Slot0.kI = 0;
        m_motorconfig.Slot0.kD = 0;
        m_motor.getConfigurator().apply(m_motorconfig);
        topLimit = new DigitalInput(0);
        bottomLimit = new DigitalInput(1);
    }

    public boolean topLimitTriggered() {
        return topLimit.get();
    }

    public boolean bottomLimitTriggered() {
        return bottomLimit.get();
    }

    public void runMotor(double speed) {
        m_motor.set(checkLimits(speed));
    }
    public void toSetpoint(double setpoint) {
        if(setpoint > 0) {
            setpoint = 0;
        } else if (setpoint < -78) {
            setpoint = -78;
        }
        m_motor.setControl(m_control.withPosition(setpoint).withSlot(0));
    }
    public double checkLimits(double speed) {
        if (topLimitTriggered()) {
            // if negative (going up) while hitting limit, don't
            if (speed < 0) {
                speed = 0;
            }
        } else if (bottomLimitTriggered()) {
            // if positive (going down) while hitting limt, don't
            if (speed > 0) {
                speed = 0;
            }
        }
        return speed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Pos", m_motor.getPosition().getValueAsDouble());
        if(bottomLimitTriggered()) {
            m_motor.setPosition(0);
        }
    }
}
