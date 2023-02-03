// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OperatorInterfaceSub extends SubsystemBase {
  boolean forward;
  int StationNum = 1;
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public OperatorInterfaceSub() {
    SmartDashboard.putBoolean("Station1", false);
    SmartDashboard.putBoolean("Station2", false);
    SmartDashboard.putBoolean("Station3", false);
    SmartDashboard.putBoolean("Station4", false);
    SmartDashboard.putBoolean("Station5", false);
    SmartDashboard.putBoolean("Station6", false);
    SmartDashboard.putBoolean("Station7", false);
    SmartDashboard.putBoolean("Station8", false);
    SmartDashboard.putBoolean("Station9", false);
  }
  public void changeState(boolean forward){
    if(forward){
      StationNum++;
    }
    else{
      StationNum--;
    }
    if(StationNum>9){
      StationNum = StationNum % 9;
    }
    if(StationNum<1){
        StationNum = 9;
      }
    System.out.println("Station"+StationNum);
    SmartDashboard.putBoolean("Station1", false);
    SmartDashboard.putBoolean("Station2", false);
    SmartDashboard.putBoolean("Station3", false);
    SmartDashboard.putBoolean("Station4", false);
    SmartDashboard.putBoolean("Station5", false);
    SmartDashboard.putBoolean("Station6", false);
    SmartDashboard.putBoolean("Station7", false);
    SmartDashboard.putBoolean("Station8", false);
    SmartDashboard.putBoolean("Station9", false);
    SmartDashboard.putBoolean("Station"+StationNum, true);


    }
  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}