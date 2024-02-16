package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    static DigitalInput topLimit;
    static DigitalInput bottomLimit;
    TalonFX m_motor;
    TalonFXConfiguration m_motorconfig;
    double output;
    PIDController pid = new PIDController(0.01, 0, 0);
    // AnalogInput m_encoder;
    AnalogEncoder m_encoder;
    double zeroPos;

    public ElevatorSubsystem() {
        m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
        m_motorconfig = new TalonFXConfiguration();
        m_motorconfig.Slot0.kG = 0.36;
        m_motorconfig.Slot0.kV = 2.38;
        m_motorconfig.Slot0.kA = 0.05;
        m_motorconfig.Slot0.kP = 0.05;
        m_motorconfig.Slot0.kI = 0;
        m_motorconfig.Slot0.kD = 0;
        m_motor.getConfigurator().apply(m_motorconfig);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void initOnce() {
        topLimit = new DigitalInput(0);
        bottomLimit = new DigitalInput(1);
        m_encoder = new AnalogEncoder(0);
        zeroPos = m_encoder.getAbsolutePosition();
    }

    public boolean topLimitTriggered() {
        return topLimit.get();
    }

    public boolean bottomLimitTriggered() {
        return bottomLimit.get();
    }

    public double closeToEndpoint(double speed) {
        if (speed < 0 && m_motor.getPosition().getValueAsDouble() < -65) {
            speed = MathUtil.clamp(speed, -0.05, 0.05);
        } else if (speed > 0 && m_motor.getPosition().getValueAsDouble() > -10) {
            speed = MathUtil.clamp(speed, -0.05, 0.05);
        }
        return speed;
    }

    public double checkEncoder(double speed) {
        if (speed < 0 && m_encoder.get() < -65) {
            speed = MathUtil.clamp(speed, -0.2, 0.2);
        } else if (speed > 0 && m_encoder.get() > -10) {
            speed = MathUtil.clamp(speed, -0.15, 0.15);
        }
        return speed;
    }

    public void runMotor(double speed) {
        speed = closeToEndpoint(speed);
        speed = capSpeed(speed);
        speed = checkLimits(speed);
        m_motor.set(speed);
    }

    public void toSetpoint(double setpoint) {
        if (setpoint > 0) {
            setpoint = 0;
        } else if (setpoint < -67) {
            setpoint = -67;
        }
        output = pid.calculate(m_motor.getPosition().getValueAsDouble(), setpoint);
        if (-0.03 < output && output < 0.03) {
            output = 0;
        }
        SmartDashboard.putNumber("Elevator PID Output", output);
        output = closeToEndpoint(output);
        output = capSpeed(output);
        output = checkLimits(output);
        runMotor(output);
    }

    public double checkLimits(double speed) {
        if (topLimitTriggered()) {
            // if negative (going up) while hitting limit, don't
            if (speed < 0) {
                speed = 0;
            }
        }
        if (bottomLimitTriggered()) {
            // if positive (going down) while hitting limt, don't
            if (speed > 0) {
                speed = 0;
            }
        }
        return speed;
    }

    public void toAmp() {
        toSetpoint(-76);
    }

    public void toSource() {
        toSetpoint(-60);
    }

    public void goDown() {
        toSetpoint(0);
    }

    public void zero() {
        if (!bottomLimitTriggered()) {
            runMotor(0.05);
        } else {
            runMotor(0);
        }
    }

    public double capSpeed(double speed) {
        return MathUtil.clamp(speed, -0.1, 0.1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Pos", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Top Limit?", topLimitTriggered());
        SmartDashboard.putBoolean("Bottom Limit?", bottomLimitTriggered());
        if (!(bottomLimit == null)) {
            if (bottomLimitTriggered()) {
                m_motor.setPosition(0);
            }
        }
        if (!(m_encoder == null)) {
            SmartDashboard.putNumber("Abs Enc", m_encoder.get());
        }
    }
}
