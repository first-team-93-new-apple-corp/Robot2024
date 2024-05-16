package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoAlignCommand extends Command {
    private Joystick m_joystick1;

    SwerveDriveSubsystem m_DriveSubsystem;
    PathPlannerPath path = PathPlannerPath.fromPathFile("AmpPathFinding");
    private Command pathfindingCommand;
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    private PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    public AutoAlignCommand(SwerveDriveSubsystem drivetrain, Joystick m_joystick1) {
        this.m_DriveSubsystem = drivetrain;
        this.m_joystick1 = m_joystick1;
    }

    // public AutoAlignCommand(SwerveDriveSubsystem drivetrain) {
    // //TODO Auto-generated constructor stub
    // }

    public Command PathFindToAmp() {
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                .1 // Rotation delay distance in meters. This is how far the robot should travel
                   // before attempting to rotate.
        );
        return pathfindingCommand;
    }

    @Override
    public void execute() {
        
            if (m_joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                        // m_DriveSubsystem.toPose(Constants.AprilTagPoseConstants.RedAmp);
                    } else {
                        // m_DriveSubsystem.toPose(Constants.AprilTagPoseConstants.BlueAmp);
                    }
                }
            } 
            // else if (m_joystick1.getRawButton(Constants.Thrustmaster.Right_Button)) {
            //     m_AutoAlignSubsystem.AutoAimTrap();
            // }
    }
}