// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;

// public class ElevatorCommand extends Command {
//     XboxController op;
//     ElevatorSubsystem m_elevator = new ElevatorSubsystem();
//     // double upSpeed = -01;
//     // double downSpeed = 0.6;
//     double setpoint = 0;
//     boolean finished = false;

//     public ElevatorCommand(XboxController op) {
//         this.op = op;
//     }
//     public ElevatorCommand() {

//     }
//     public void initOnce() {
//         m_elevator.initOnce();
//     }
//     public void preflight() {
//         if (!m_elevator.bottomLimitTriggered()) {
//             m_elevator.zero();
//         }
//         finished = true;
//     }
//     public void toSetpoint(double setpoint) {
//         m_elevator.toSetpoint(setpoint);
//     }
//     // public boolean isFinished() {
//     //     return finished;
//     // }
//     @Override
//     public void execute() {
//         if (op.getPOV() == 0) {
//             m_elevator.toSource();
//             // m_elevator.runMotor(-0.1);
//         } else if (op.getPOV() == 90) {
//             m_elevator.toAmp();
//         } else if (op.getPOV() == 180) {
//             m_elevator.goDown();
//             // m_elevator.runMotor(0.1);
//         } else if (op.getPOV() == 270) {
//             // ELevator out for balancing on the chain (This broke it last time for some odd
//             // reason)
//             // m_elevator.toSetpoint(-15);
//         } else {
//             // m_elevator.runMotor(0);
//         }
//     }
// }
