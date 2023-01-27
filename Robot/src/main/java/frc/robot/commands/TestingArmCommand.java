// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;

public class TestingArmCommand extends CommandBase {


    TelescopingSubsystem m_TelescopingSubsystem; 


    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public TestingArmCommand(TelescopingSubsystem m_TelescopingSubsystem) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem; 

        addRequirements(m_TelescopingSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_TelescopingSubsystem.OscilateArm();
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
