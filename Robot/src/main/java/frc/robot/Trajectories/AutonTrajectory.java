// package frc.robot.Trajectories;
//TODO Fix Later

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.FerbConfig;
// import frc.robot.commands.NewRamsetteCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class AutonTrajectory {

//   public AutonTrajectory() {}

//   public static Command getCommand(
//     DrivetrainSubsystem m_drive,
//     Trajectory trajectory
//   ) {
//     // probably should uncomment when everything gets screwed up at comps....that wont happen though right?
//     // RamseteController disabledRamsete = new RamseteController() {

//     //   @Override
//     //   public ChassisSpeeds calculate(
//     //     Pose2d currentPose,
//     //     Pose2d poseRef,
//     //     double linearVelocityRefMeters,
//     //     double angularVelocityRefRadiansPerSecond
//     //   ) {
//     //     return new ChassisSpeeds(
//     //       linearVelocityRefMeters,
//     //       0.0,
//     //       angularVelocityRefRadiansPerSecond
//     //     );
//     //   }
//     // };

//     PIDController leftController = new PIDController(
//       FerbConfig.kPDriveVel,
//       0,
//       0
//     );
//     PIDController rightController = new PIDController(
//       FerbConfig.kPDriveVel,
//       0,
//       0
//     );

//     NewRamsetteCommand ramseteCommand = new NewRamsetteCommand(
//       trajectory,
//       m_drive::getPose,
//       new RamseteController(),

//       //just in case huh? 
//       // disabledRamsete,
//       new SimpleMotorFeedforward(
//         FerbConfig.ksVolts,
//         FerbConfig.kvVoltSecondsPerMeter,
//         FerbConfig.kaVoltSecondsSquaredPerMeter
//       ),
//       FerbConfig.kDriveKinematics,
//       m_drive::getWheelSpeeds,
//       leftController,
//       rightController,
//       (leftVolts, rightVolts) -> {
//         m_drive.tankDriveVolts(leftVolts, rightVolts);
//       },
//       m_drive
//     );

//     return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
//   }
// }
