package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.SignalLoggerJNI;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Preflight;

public class ElevatorSubsystem extends SubsystemBase {
    static DigitalInput topLimit;
    static DigitalInput bottomLimit;
    TalonFX m_motor;
    // TalonFXConfiguration m_motorconfig;
    double output;
    PIDController pid = new PIDController(0.075, 0, 0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    // AnalogInput m_encoder;
    // AnalogEncoder m_encoder;
    double zeroPos;
    double rawoutput;
    double currentPos;
    double setpoint = 0;
    double highSetpoint = -75;
    double lowSetpoint = -3;

    public enum elevatorState {
        HoldState,
        ToSetpoint,
        Zeroing,
        TempState,
        Init,
        Disabled
    }

    public elevatorState currentState;

    public ElevatorSubsystem() {
        m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
        config.CurrentLimits.SupplyCurrentLimit = 15;
        config.CurrentLimits.SupplyCurrentThreshold = 0;
        config.CurrentLimits.SupplyTimeThreshold = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
        currentState = elevatorState.Init;
    }

    public void initOnce() {
        if (currentState == elevatorState.Init) {
            if (topLimit == null) {
                topLimit = new DigitalInput(0);
            }
            if (bottomLimit == null) {
                bottomLimit = new DigitalInput(1);
            }
            currentState = elevatorState.HoldState;
        } else {
            System.out.println("Elevator has already been initialized!");
        }
    }

    public boolean topLimitTriggered() {
        if (!(topLimit == null)) {
            return topLimit.get();
        } else {
            return true;
        }
    }

    public boolean bottomLimitTriggered() {
        if (!(bottomLimit == null)) {
            return bottomLimit.get();
        } else {
            return true;
        }
    }

    public void runElevator() {
        switch (currentState) {
            default:
            case HoldState:
                if (Preflight.isPreflightDone()) {
                    m_motor.set(0);
                    m_motor.setNeutralMode(NeutralModeValue.Brake);
                } else {
                    System.out.println("Preflight not completed. Disabling elevator.");
                    currentState = elevatorState.Disabled;
                }
                break;

            case TempState:
                if (topLimitTriggered()) {
                    m_motor.set(-0.05);
                } else if (bottomLimitTriggered()) {
                    m_motor.set(0.05);
                } else {
                    m_motor.set(0);
                }
                break;

            case ToSetpoint:
                currentPos = m_motor.getPosition().getValueAsDouble();
                output = pid.calculate(currentPos, setpoint);
                // If we are hitting top/bottom limit and trying to break the robot, go to
                // stopped state
                if ((output < 0 && topLimitTriggered()) || (output > 0 && bottomLimitTriggered())) {
                    currentState = elevatorState.HoldState;
                    // currentState = elevatorState.TempState;
                } else {
                    if (currentPos <= -65 || currentPos >= -15) {
                        output = MathUtil.clamp(output, -0.6, 0.6);
                    }
                    output = MathUtil.clamp(output, -0.90, 0.90);
                    m_motor.set(output);
                }
                break;
            case Zeroing:
                if (!bottomLimitTriggered()) {
                    m_motor.set(0.05);
                } else {
                    m_motor.set(0);
                    m_motor.setPosition(0);
                    currentState = elevatorState.HoldState;
                }
                break;

            case Init:
                initOnce();
                break;

            case Disabled:
                m_motor.set(0);
                m_motor.setNeutralMode(NeutralModeValue.Coast);
                break;
        }
    }

    public void toSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
        setpoint = MathUtil.clamp(setpoint, highSetpoint, lowSetpoint);
        if (!(currentState == elevatorState.Disabled)) {
            currentState = elevatorState.ToSetpoint;
        } else {
            System.out.println("Elevator is disabled!");
        }
    }

    public void zero() {
        if (!(currentState == elevatorState.Disabled)) {
            currentState = elevatorState.Zeroing;
        } else {
            System.out.println("Elevator is disabled!");
        }
    }

    public TalonFX getMotor() {
        return m_motor;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Pos", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Top Limit?", topLimitTriggered());
        SmartDashboard.putBoolean("Bottom Limit?", bottomLimitTriggered());

        SignalLogger.writeDouble("Elevator Pos", m_motor.getPosition().getValueAsDouble());
        SignalLogger.writeBoolean("Top Limit?", topLimitTriggered());
        SignalLogger.writeBoolean("Bottom Limit?", bottomLimitTriggered());

        runElevator();
        // if (!(bottomLimit == null) && bottomLimitTriggered()) {
        // m_motor.setPosition(0);
        // }
    }
}
