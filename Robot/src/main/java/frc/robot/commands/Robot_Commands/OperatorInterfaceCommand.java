// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Robot_Commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.WristSubsystem;


public class OperatorInterfaceCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public OperatorInterfaceCommand(boolean forward) {
    ShuffleboardTab OperatorTab = Shuffleboard.getTab("DriverInterface");
    OperatorTab.add("Station1", true).withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    int StationNum = 1;
    SmartDashboard.putBoolean("Station1", false);
    SmartDashboard.putBoolean("Station2", false);
    SmartDashboard.putBoolean("Station3", false);
    SmartDashboard.putBoolean("Station4", false);
    SmartDashboard.putBoolean("Station5", false);
    SmartDashboard.putBoolean("Station6", false);
    SmartDashboard.putBoolean("Station7", false);
    SmartDashboard.putBoolean("Station8", false);
    SmartDashboard.putBoolean("Station9", false);

    
    addRequirements();



  }

  @Override
  public void initialize() {


  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
