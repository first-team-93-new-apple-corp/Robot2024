// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_DriveSubsystem;
    PathPlannerTrajectory straightWithATwist;
    Timer timer;
    double endTime;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoCommand(DriveSubsystem subsystem) {
        straightWithATwist = PathPlanner.loadPath("StraightWithATwist", 3, 3);
        m_DriveSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
        endTime = straightWithATwist.getTotalTimeSeconds();

        m_DriveSubsystem.resetOdometry(straightWithATwist.getInitialHolonomicPose());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PathPlannerState curState = (PathPlannerState) straightWithATwist.sample(timer.get());
        m_DriveSubsystem.driveAuton(curState);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.drive(0, 0, 0, false, DriveConstants.Center);
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get()>=endTime;
    }
}
