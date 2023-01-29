// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;

public class ManualTelescopeCommand extends CommandBase {


    TelescopingSubsystem m_TelescopingSubsystem; 
    double speed;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ManualTelescopeCommand(TelescopingSubsystem m_TelescopingSubsystem,double speed) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem; 
        this.speed = speed;
        addRequirements(m_TelescopingSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_TelescopingSubsystem.directMotorCommand(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_TelescopingSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
