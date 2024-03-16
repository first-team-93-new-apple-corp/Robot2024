package frc.robot.subsystems.Elevator.IO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.SimUtilities.MotorSim;
import frc.robot.commands.Preflight;

public class ElevatorIOSim implements ElevatorIO {
    static DigitalInput topLimit;
    static DigitalInput bottomLimit;
    private MotorSim ElevatorMotor;
    private double output;
    private PIDController pid = new PIDController(0.075, 0, 0);

    private double zeroPos;
    private double rawoutput;
    private double currentPos;
    private double setpoint;
    private double highSetpoint;
    private double lowSetpoint;

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
        ElevatorMotor = new MotorSim(100);
        currentState = elevatorState.Init;
        setpoint = constants.setpoint;
        highSetpoint = constants.highSetpoint;
        lowSetpoint = constants.lowSetpoint;
    }
    @Override
    public void updateValues(ElevatorIOInputs inputs){
        ElevatorMotor.periodic();
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
            return false;
        }
    }

    public boolean bottomLimitTriggered(){
        if (!(bottomLimit == null)) {
            return bottomLimit.get();
        } else {
            return false;
        }
    }

    public double ElevatorPosition(){
        return ElevatorMotor.getDistance();
    }

    public void runElevator(){
        switch (currentState) {
            default:
            case HoldState:
                if (Preflight.isPreflightDone()) {
                    ElevatorMotor.set(0);
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
                currentPos = ElevatorMotor.getDistance();
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
                    ElevatorMotor.set(0.05);
                } else {
                    ElevatorMotor.set(0);
                    ElevatorMotor.setPosition(0);
                    currentState = elevatorState.HoldState;
                }
                break;

            case Init:
                initOnce();
                break;

            case Disabled:
                ElevatorMotor.set(0);
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
