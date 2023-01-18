package frc.robot.commands;

import java.util.logging.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevellingCommand extends CommandBase {

    DriveSubsystem m_DriveSubsystem;

    PIDController LevellingPID;

    public AutoLevellingCommand(DriveSubsystem m_DriveSubsystem) {

        this.m_DriveSubsystem = m_DriveSubsystem;

        LevellingPID = new PIDController(0, 0, 0);
        LevellingPID.setTolerance(1);


        addRequirements(m_DriveSubsystem);

    }

    @Override
    public void initialize() {

        LevellingPID.setSetpoint(0);
    }

    double MotorCommand;

    @Override
    public void execute() {
        MotorCommand = LevellingPID.calculate(m_DriveSubsystem.getLevel());

        System.out.println(MotorCommand);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
