// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;


public class ArmCommandManual extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 public ShoulderSubsystem m_ShoulderSubsystem;
 public TelescopingSubsystem m_TelescopingSubsystem;
 Boolean Up;
 enum ArmState {
  DEFAULT_STATE,
  GROUND_LOAD,
  PLAYER_LOAD,
  MID_CONE,
  MID_CUBE,
  HIGH_CUBE,
  HIGH_CONE,
  LOW_HYBRID
}
  HashMap<ArmCommandManual.ArmState, double[]> Positions = new HashMap<ArmCommandManual.ArmState, double[]>();
  public ArmCommandManual(boolean Up,
      TelescopingSubsystem m_TelescopingSubsystem, ShoulderSubsystem m_ShoulderSubsystem) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;
    addRequirements(m_ShoulderSubsystem,m_TelescopingSubsystem);
    this.Up = Up;


  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(Up){
        if(m_ShoulderSubsystem.getDegrees()+0.25<Constants.MaxAngle){ //TODO Needs to flip when goes over center, probably a state machine
            m_ShoulderSubsystem.RunShoulderMotors(m_ShoulderSubsystem.getDegrees()+0.25); //+ 0.25 degrees every command loop
        }
        else{
            m_ShoulderSubsystem.RunShoulderMotors(m_ShoulderSubsystem.getDegrees());
        }
    }
    else{
        if(m_ShoulderSubsystem.getDegrees()-0.25<Constants.MinAngle){ //TODO Needs to flip when goes over center, probably a state machine
            m_ShoulderSubsystem.RunShoulderMotors(m_ShoulderSubsystem.getDegrees()-0.25);
        }
        else{
            m_ShoulderSubsystem.RunShoulderMotors(m_ShoulderSubsystem.getDegrees());
        }
    }
    
}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}