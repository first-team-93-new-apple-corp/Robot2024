package frc.robot.subsystems.Elevator.IO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Preflight;

public class ElevatorIOSim implements ElevatorIO {
    static DigitalInput topLimit;
    static DigitalInput bottomLimit;
    TalonFX ElevatorMotor;
    double output;
    PIDController pid = new PIDController(0.075, 0, 0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration zeroConfig = new TalonFXConfiguration();

    double zeroPos;
    double rawoutput;
    double currentPos;
    double setpoint;
    double highSetpoint;
    double lowSetpoint;

    public enum elevatorState {
        HoldState,
        ToSetpoint,
        Zeroing,
        TempState,
        Init,
        Disabled
    }

    public elevatorState currentState;


    public ElevatorIOSim(ElevatorConstants constants) {
        ElevatorMotor = new TalonFX(constants.ElevatorMotor, "rio");
        config.CurrentLimits.SupplyCurrentLimit = 15;
        config.CurrentLimits.SupplyCurrentThreshold = 0;
        config.CurrentLimits.SupplyTimeThreshold = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ElevatorMotor.getConfigurator().apply(config);
        currentState = elevatorState.Init;
        zeroConfig.CurrentLimits.SupplyCurrentLimit = 5;
        zeroConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        setpoint = constants.setpoint;
        highSetpoint = constants.highSetpoint;
        lowSetpoint = constants.lowSetpoint;
    }
    @Override
    public void updateValues(ElevatorIOInputs inputs){
        
    }

    public void disable(){
        // currentState = elevatorState.Disabled;
    }

    public void initOnce(){
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

    public boolean topLimitTriggered(){
        if (!(topLimit == null)) {
            return topLimit.get();
        } else {
            return true;
        }
    }

    public boolean bottomLimitTriggered(){
        if (!(bottomLimit == null)) {
            return bottomLimit.get();
        } else {
            return true;
        }
    }

    public double ElevatorPosition(){
        return ElevatorMotor.getPosition().getValueAsDouble();
    }

    public void runElevator(){
        switch (currentState) {
            default:
            case HoldState:
                if (Preflight.isPreflightDone()) {
                    ElevatorMotor.set(0);
                    ElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
                } else {
                    System.out.println("Preflight not completed. Disabling elevator.");
                    currentState = elevatorState.Disabled;
                }
                break;

            case TempState:
                if (topLimitTriggered()) {
                    ElevatorMotor.set(-0.05);
                } else if (bottomLimitTriggered()) {
                    ElevatorMotor.set(0.05);
                } else {
                    ElevatorMotor.set(0);
                }
                break;

            case ToSetpoint:
                currentPos = ElevatorMotor.getPosition().getValueAsDouble();
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
                    ElevatorMotor.set(output);
                }
                break;
            case Zeroing:
                if (!bottomLimitTriggered()) {
                    ElevatorMotor.getConfigurator().apply(zeroConfig);
                    ElevatorMotor.set(0.05);
                } else {
                    ElevatorMotor.set(0);
                    ElevatorMotor.setPosition(0);
                    ElevatorMotor.getConfigurator().apply(config);
                    currentState = elevatorState.HoldState;
                }
                break;

            case Init:
                initOnce();
                break;

            case Disabled:
                ElevatorMotor.set(0);
                ElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
                break;
        }
    }


    public void toSetpoint(double newSetpoint){
        setpoint = newSetpoint;
        setpoint = MathUtil.clamp(setpoint, highSetpoint, lowSetpoint);
        currentState = elevatorState.ToSetpoint;
    }

    public void zero() {
        currentState = elevatorState.Zeroing;
    }

}
