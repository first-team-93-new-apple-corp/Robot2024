// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.ElevatorSubsystem;

// import com.ctre.phoenix6.hardware.*;

// public class ClimbingLevel extends Command {
//     ClimberCommand m_climberCommand;
//     ElevatorCommand m_ElevatorCommand;
//     // ElevatorSubsystem m_ElevatorSubsystem;
//     Pigeon2 m_pigeon2;
//     double elevatorSetpoint = -20.0;
//     Robot m_robot;
//     int pidValue;
//     PIDController elevatorPID = new PIDController(pidValue, 0, 0);

//     public ClimbingLevel(
//             ClimberCommand m_climberCommand,
//             ElevatorCommand m_ElevatorCommand) {
//         // Use addRequirements() here to declare subsystem dependencies.
//         this.m_climberCommand = m_climberCommand;
//         this.m_ElevatorCommand = m_ElevatorCommand;
//         m_robot = new Robot();
//         m_pigeon2 = m_robot.getPigeon();

//         /*
//          * I don't think that it is needed atm but we could take the time to implement
//          * 
//          * as a like precaution?
//          * 
//          * looked over the code and elevator already has PIDs in the toSetpoint method
//          * in the Subsystem
//          * 
//          */
//         if (m_pigeon2.getRoll().getValueAsDouble() > 0.5) {
//             m_climberCommand.changeLeft(-5);
//             m_climberCommand.changeRight(5);
//         } else if (m_pigeon2.getRoll().getValueAsDouble() < -0.5) {
//             m_climberCommand.changeLeft(5);
//             m_climberCommand.changeRight(-5);
//         }
//         // This doesn't have to change since the weight forward/backwards should always
//         // be the same
//         m_ElevatorCommand.toSetpoint(elevatorSetpoint);

//         // // Auto elevating code to be tested after manual elevating is tested
//         // if (m_pigeon2.getPitch().getValueAsDouble() < 1.5) {
//         //     m_ElevatorCommand.toSetpoint(elevatorSetpoint);
//         // }
//     }
// }
