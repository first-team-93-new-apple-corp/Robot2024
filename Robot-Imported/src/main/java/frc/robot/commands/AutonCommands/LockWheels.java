package frc.robot.commands.AutonCommands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LockWheels extends Command {

    DriveSubsystem m_DriveSubsystem;

    PIDController LevellingPID;

    public LockWheels(DriveSubsystem m_DriveSubsystem) {

        this.m_DriveSubsystem = m_DriveSubsystem;


        addRequirements(m_DriveSubsystem);

    }

    @Override
    public void initialize() {
        m_DriveSubsystem.lockWheels();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        
        return true; 

    }
}
