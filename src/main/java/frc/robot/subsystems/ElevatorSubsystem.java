// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.AnalogEncoder;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ElevatorSubsystem extends SubsystemBase {
//     static DigitalInput topLimit;
//     static DigitalInput bottomLimit;
//     TalonFX m_motor;
//     // TalonFXConfiguration m_motorconfig;
//     double output;
//     PIDController pid = new PIDController(0.075, 0, 0);
//     // AnalogInput m_encoder;
//     AnalogEncoder m_encoder;
//     double zeroPos;
//     double rawoutput;
//     double currentPos;
//     double setpoint = 0;
//     double highSetpoint = -75;
//     double lowSetpoint = -3;
//     public enum elevatorState {
//         HoldState,
//         ToSetpoint,
//         Zeroing,
//         TempState,
//         Init
//     }

//     public elevatorState currentState;

//     public ElevatorSubsystem() {
//         m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
//         m_motor.setNeutralMode(NeutralModeValue.Brake);
//         currentState = elevatorState.Init;
//     }

//     public void initOnce() {
//         if (currentState == elevatorState.Init) {
//             topLimit = new DigitalInput(0);
//             bottomLimit = new DigitalInput(1);
//             m_encoder = new AnalogEncoder(0);
//             zeroPos = m_encoder.getAbsolutePosition();
//             currentState = elevatorState.HoldState;
//         } else {
//             System.out.println("Elevator has already been initialized!");
//         }
//     }

//     public boolean topLimitTriggered() {
//         return topLimit.get();
//     }

//     public boolean bottomLimitTriggered() {
//         return bottomLimit.get();
//     }

//     public void runElevator() {
//         switch (currentState) {
//             default:
//             case HoldState:
//                 m_motor.set(0);
//                 break;

//             case TempState:
//                 if (topLimitTriggered()) {
//                     m_motor.set(-0.05);
//                 } else if (bottomLimitTriggered()) {
//                     m_motor.set(0.05);
//                 } else {
//                     m_motor.set(0);
//                 }
//                 break;

//             case ToSetpoint:
//                 currentPos = m_motor.getPosition().getValueAsDouble();
//                 output = pid.calculate(currentPos, setpoint);
//                 // If we are hitting top/bottom limit and trying to break the robot, go to
//                 // stopped state
//                 if ((output < 0 && topLimitTriggered()) || (output > 0 && bottomLimitTriggered())) {
//                     currentState = elevatorState.HoldState;
//                     // currentState = elevatorState.TempState;
//                 } else {
//                     if (currentPos <= -65 || currentPos >= -15) {
//                         output = MathUtil.clamp(output, -0.3, 0.3);
//                     }
//                     output = MathUtil.clamp(output, -0.75, 0.75);
//                     m_motor.set(output);
//                 }
//                 break;
//             case Zeroing:
//                 if (!bottomLimitTriggered()) {
//                     m_motor.set(0.05);
//                 } else {
//                     m_motor.set(0);
//                     currentState = elevatorState.HoldState;
//                 }
//                 break;

//             case Init:
//                 initOnce();
//                 break;
//         }
//     }

//     public void toSetpoint(double newSetpoint) {
//         setpoint = newSetpoint;
//         setpoint = MathUtil.clamp(setpoint, highSetpoint, lowSetpoint);
//         currentState = elevatorState.ToSetpoint;
//     }
//     public void zero() {
//         currentState = elevatorState.Zeroing;
//     }
//     public TalonFX getMotor() {
//         return m_motor;
//     }
//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Elevator Pos",
//                 m_motor.getPosition().getValueAsDouble());
//         SmartDashboard.putBoolean("Top Limit?", topLimitTriggered());
//         SmartDashboard.putBoolean("Bottom Limit?", bottomLimitTriggered());
//         runElevator();
//         // if (!(bottomLimit == null) && bottomLimitTriggered()) {
//         //     m_motor.setPosition(0);
//         // }
//     }
// }
