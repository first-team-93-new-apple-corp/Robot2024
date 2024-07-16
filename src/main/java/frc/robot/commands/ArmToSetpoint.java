package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ARM_SETPOINTS;
import frc.robot.subsystems.Helpers.ArmHelper;

public class ArmToSetpoint extends Command {
    ARM_SETPOINTS desiredState;

    ArmHelper m_HelperSubsystem;
    boolean isAuto;

    public ArmToSetpoint(
            ArmHelper m_HelperSubsystem,
            ARM_SETPOINTS desiredState) {
        this(m_HelperSubsystem, desiredState, false);
    }

    public ArmToSetpoint(
            ArmHelper m_HelperSubsystem,
            ARM_SETPOINTS desiredState,
            boolean isAuto) {
        this.m_HelperSubsystem = m_HelperSubsystem;
        this.desiredState = desiredState;
        this.isAuto = isAuto;
        addRequirements(m_HelperSubsystem.getRequiredSubsystems());
    }

    @Override
    public void initialize() {
        m_HelperSubsystem.resetSubsystem(desiredState);
    }

    @Override
    public void execute() {
        m_HelperSubsystem.follow();
    }

}
