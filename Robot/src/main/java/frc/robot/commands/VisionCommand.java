package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
    VisionSubsystem m_VisionSubsystem;

    public VisionCommand(VisionSubsystem m_VisionSubsystem) {
        m_VisionSubsystem = new VisionSubsystem("limelight-front");
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_VisionSubsystem = new VisionSubsystem("limelight-front");


            m_VisionSubsystem.updateValues();
            m_VisionSubsystem.followTape();
    
    }

    // @Override
    // public boolean isFinished() {
    // return false;
    // }
}