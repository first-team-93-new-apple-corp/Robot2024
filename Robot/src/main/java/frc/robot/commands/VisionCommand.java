package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsytem;

public class VisionCommand extends CommandBase {
    VisionSubsytem m_VisionSubsytem;

    @Override
    public void initialize() {
        m_VisionSubsytem = new VisionSubsytem("limelight-front");
    }

    @Override
    public void execute() {
        m_VisionSubsytem.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
