// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.HashMap;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ShoulderSubsystem;
// import frc.robot.subsystems.TelescopingSubsystem;

// public class ShoulderCommand extends CommandBase {


//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     public ShoulderCommand(ShoulderSubsystem m_ShoulderSubsystem) {

//         addRequirements(m_ShoulderSubsystem);

//         SmartDashboard.putNumber("Arm Setpoint", 0);
//     }

//     @Override
//     public void initialize() {

//     }

//     enum ArmState {
//         ZEROING,
//         STANBY,
//         SETPOINT

//     }

//     ArmState currentArmState = ArmState.ZEROING;

//     @Override
//     public void execute() {
//         // m_TelescopingSubsystem.OscilateArm();
//         // m_TelescopingSubsystem.directMotorCommand(0.1);

//         switch (currentArmState) {
//             case STANBY:
//                 if (SmartDashboard.getNumber("Arm Setpoint", -1) != 0) {
//                     currentArmState = ArmState.SETPOINT;
//                 }
//                 break;
//             case SETPOINT:
//                 m_TelescopingSubsystem.toSetpoint(SmartDashboard.getNumber("Arm Setpoint", 0));

//                 break;
//             case ZEROING:
//                 if (!(m_TelescopingSubsystem.getTicks() <= 140)) {
//                     m_TelescopingSubsystem.directMotorCommand(-0.1);
//                 } else {
//                     m_TelescopingSubsystem.directMotorCommand(0);
//                     currentArmState = ArmState.STANBY;
//                 }
//                 break;
//             default:
//                 break;

//         }

//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_TelescopingSubsystem.stopMotor();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }