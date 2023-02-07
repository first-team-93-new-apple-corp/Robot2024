package frc.robot.commands.Telescope_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingSubsystem;

public class Tuning_TelescopeCommand extends CommandBase {

    TelescopingSubsystem m_TelescopingSubsystem;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public Tuning_TelescopeCommand(TelescopingSubsystem m_TelescopingSubsystem) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;

        addRequirements(m_TelescopingSubsystem);

        SmartDashboard.putNumber("Telescope Setpoint", 181);
    }

    @Override
    public void initialize() {

    }

    enum ArmState {
        ZEROING,
        STANBY,
        SETPOINT

    }

    ArmState currentArmState = ArmState.STANBY;

    @Override
    public void execute() {
        switch (currentArmState) {
            case STANBY:
                if (SmartDashboard.getNumber("Telescope Setpoint", 181) != 0) {
                    currentArmState = ArmState.SETPOINT;
                }
                break;
            case SETPOINT:
            m_TelescopingSubsystem.toSetpoint(SmartDashboard.getNumber("Telescope Setpoint", 181));

                break;
            case ZEROING:
                if (!(m_TelescopingSubsystem.getTicks() <= 140)) {
                    m_TelescopingSubsystem.directMotorCommand(-0.1);
                } else {
                    m_TelescopingSubsystem.directMotorCommand(0);
                    currentArmState = ArmState.STANBY;
                }
                break;
            default:
                break;

        }

    }

    @Override
    public void end(boolean interrupted) {
        m_TelescopingSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
