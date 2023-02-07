package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevellingCommand extends CommandBase {

    DriveSubsystem m_DriveSubsystem;

    PIDController LevellingPID;
    Timer eliasTimer;

    final double TimeAtLevel = 0.5; 

    public AutoLevellingCommand(DriveSubsystem m_DriveSubsystem) {

        this.m_DriveSubsystem = m_DriveSubsystem;

        LevellingPID = new PIDController(0.01, 0.007, 0.002);
        LevellingPID.setTolerance(1);

        eliasTimer = new Timer();

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

        if (MotorCommand > 0.3) {
            MotorCommand = 0.3;
        }
        m_DriveSubsystem.drive(MotorCommand, 0, 0, false, DriveConstants.Center);

    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.drive(0, 0, 0, false, DriveConstants.Center);

    }

    @Override
    public boolean isFinished() {
        if (LevellingPID.atSetpoint()) {


            eliasTimer.start();
        } else { 
            eliasTimer.stop();
            eliasTimer.reset();
        }


        return eliasTimer.advanceIfElapsed(TimeAtLevel);

    }
}
