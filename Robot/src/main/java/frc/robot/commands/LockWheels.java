package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LockWheels extends CommandBase {

    DriveSubsystem m_DriveSubsystem;

    PIDController LevellingPID;

    Timer m_Timer;

    public LockWheels(DriveSubsystem m_DriveSubsystem) {

        this.m_DriveSubsystem = m_DriveSubsystem;

        m_Timer = new Timer();

        addRequirements(m_DriveSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_DriveSubsystem.lockWheels();

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return m_Timer.advanceIfElapsed(0.5);

    }
}