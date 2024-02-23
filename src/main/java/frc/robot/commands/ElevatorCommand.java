// package frc.robot.commands;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.ElevatorSubsystem;

// public class ElevatorCommand extends Command {
//     XboxController op;
//     ElevatorSubsystem m_elevator = new ElevatorSubsystem();
//     double setpoint = 0;
//     double ampSetpoint = -75;
//     double sourceSetpoint = -75;

//     public ElevatorCommand(XboxController op) {
//         this.op = op;
//     }

//     public void initOnce() {
//         m_elevator.initOnce();
//     }

//     public void preflight() {
//         m_elevator.zero();
//     }

//     public TalonFX getMotor() {
//         return m_elevator.getMotor();
//     }

//     @Override
//     public void execute() {
//         if (op.getPOV() == 0) {
//             m_elevator.toSetpoint(-75);
//         } else if (op.getPOV() == 90) {
//             m_elevator.toSetpoint(-40);
//         } else if (op.getPOV() == 180) {
//             m_elevator.toSetpoint(-3);
//         } else if (op.getPOV() == 270) {
//             m_elevator.toSetpoint(-25);
//         } else {

//         }

//         if (op.getRawButton(Constants.xbox.RightShoulderButton)) {
//             m_elevator.toSetpoint(ampSetpoint);
//         }

//         if (op.getRawButton(Constants.xbox.LeftShoulderButton)) {
//             m_elevator.toSetpoint(sourceSetpoint);
//         }
//     }
// }
