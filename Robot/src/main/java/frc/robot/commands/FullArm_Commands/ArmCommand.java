// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.FullArm_Commands;

// import java.util.HashMap;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.GrabberSubsystem;
// import frc.robot.subsystems.ShoulderSubsystem;
// import frc.robot.subsystems.TelescopingSubsystem;
// import frc.robot.subsystems.WristSubsystem;

// public class ArmCommand extends CommandBase {

//   @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//   public ShoulderSubsystem m_ShoulderSubsystem;
//   public TelescopingSubsystem m_TelescopingSubsystem;
//   GrabberSubsystem m_GrabberSubsystem;
//   WristSubsystem m_WristSubsystem;
//   ArmState desiredState;

//   enum ArmState {
//     DEFAULT_STATE,
//     GROUND_LOAD,
//     PLAYER_LOAD,
//     MID_CONE,
//     MID_CUBE,
//     HIGH_CUBE,
//     HIGH_CONE,
//     LOW_HYBRID
//   }

//   HashMap<ArmCommand.ArmState, double[]> Positions = new HashMap<ArmCommand.ArmState, double[]>(); // TODO change
//                                                                                                    // hashmap to include
//                                                                                                    // positions/speeds
//                                                                                                    // of all subsystems
//                                                                                                    // for different
//                                                                                                    // setpoints

//   public ArmCommand(ArmState state,
//       TelescopingSubsystem m_TelescopingSubsystem, ShoulderSubsystem m_ShoulderSubsystem,
//       GrabberSubsystem m_GrabberSubsystem, WristSubsystem m_WristSubsystem, ArmState Position) {
//     this.m_ShoulderSubsystem = m_ShoulderSubsystem;
//     this.m_TelescopingSubsystem = m_TelescopingSubsystem;
//     this.m_GrabberSubsystem = m_GrabberSubsystem;
//     this.m_WristSubsystem = m_WristSubsystem;

//     addRequirements(m_ShoulderSubsystem, m_TelescopingSubsystem, m_GrabberSubsystem, m_WristSubsystem);

//   }

//   @Override
//   public void initialize() {

//   }

//   @Override
//   public void execute() {
//     m_ShoulderSubsystem.toSetpoint(0);
//     // should go brr fr.
//     if (m_ShoulderSubsystem.getDegrees() > 60.0) { // guarantee that we won't clip anything

//       m_TelescopingSubsystem.toSetpoint(0);
//       // telescoping subsystem should only move once the arm is outside of the
//       // robotframe/ won't clip with the robot.

//       m_WristSubsystem.toSetpoint(0);
//       // wrist subsystem should only move once the arm is outside of the robot
//       // frame/won't clip with the robot.

//       m_GrabberSubsystem.directMotorCommand(0);
//       // grabber needs to have logic for when a game piece is aquired, how should it
//       // actuate, how should the wheels run, etc.
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
