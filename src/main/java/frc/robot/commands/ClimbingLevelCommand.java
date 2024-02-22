package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TunerConstants;

import com.ctre.phoenix6.hardware.*;
public class ClimbingLevelCommand extends Command {
    ClimberCommand m_climberCommand;
    // ElevatorCommand m_ElevatorCommand;
    // ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    Pigeon2 m_pigeon2;
    double elevatorSetpoint = -20.0;
    private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
    public ClimbingLevelCommand(
            ClimberCommand m_climberCommand) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_climberCommand = m_climberCommand;
        // m_ElevatorCommand = new ElevatorCommand();
        m_pigeon2 = drivetrain.getPigeon2();

        /*
         * I don't think that it is needed atm but we could take the time to implement

         as a like precaution?

         looked over the code and elevator already has PIDs in the toSetpoint method in the Subsystem
         
         */
        if(m_pigeon2.getRoll().getValueAsDouble() > 0.5) {
            m_climberCommand.changeLeft(-5);
            m_climberCommand.changeRight(5);
        } else if(m_pigeon2.getRoll().getValueAsDouble() < -0.5) {
            m_climberCommand.changeLeft(5);
            m_climberCommand.changeRight(-5);
        }
        //This doesn't have to change since the weight forward/backwards should always be the same
        // m_ElevatorSubsystem.toSetpoint(elevatorSetpoint);
    }
    public Command levelCommand () {
        return this;
    }
}
