// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;


public class ArmCommandAutomated extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 public ShoulderSubsystem m_ShoulderSubsystem;
 public TelescopingSubsystem m_TelescopingSubsystem;
 ArmState desiredState;
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
  HashMap<ArmCommandAutomated.ArmState, double[]> Positions = new HashMap<ArmCommandAutomated.ArmState, double[]>();
  public ArmCommandAutomated(ArmState state,
      TelescopingSubsystem m_TelescopingSubsystem, ShoulderSubsystem m_ShoulderSubsystem) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;
    addRequirements(m_ShoulderSubsystem,m_TelescopingSubsystem);
    Positions.put(ArmState.DEFAULT_STATE, new double[] {0.0,0.0});
    Positions.put(ArmState.GROUND_LOAD, new double[] {0.0,0.0});
    Positions.put(ArmState.PLAYER_LOAD, new double[] {0.0,0.0});
    Positions.put(ArmState.MID_CONE, new double[] {0.0,0.0});
    Positions.put(ArmState.MID_CUBE, new double[] {0.0,0.0});
    Positions.put(ArmState.HIGH_CUBE, new double[] {0.0,0.0});
    Positions.put(ArmState.HIGH_CONE, new double[] {0.0,0.0});
    Positions.put(ArmState.LOW_HYBRID, new double[] {0.0,0.0}); //First Param = Shoulder angle, second param = telescoping distance;
    desiredState = state;


  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_ShoulderSubsystem.RunShoulderMotors(Positions.get(desiredState)[0]);
    m_TelescopingSubsystem.RunTelescopingMotors(Positions.get(desiredState)[1]);
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
