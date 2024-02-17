package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX frontIntake;
    private TalonFX backIntake;
    TimeOfFlight midTOF;
    private double IntakeSpeed = 0.75;
    private double PassoverSpeed = 0.5;
    private ShooterSubsystem m_shooter;

    public enum intakeState {
        Stage1,
        Stage2,
        Stage3
    }

    private intakeState state = intakeState.Stage1;

    public IntakeSubsystem(ShooterSubsystem m_shooter) {
        this.m_shooter = m_shooter;
        frontIntake = new TalonFX(Constants.CTRE.RIO.F_Intake, "rio");
        backIntake = new TalonFX(Constants.CTRE.RIO.B_Intake, "rio");
        backIntake.setInverted(false);
        midTOF = new TimeOfFlight(22);
        midTOF.setRangingMode(RangingMode.Short, 24);
    }

    public Command AutonStopIntake() {
        return this.runOnce(() -> stop());
    }

    public void Intake() {
        switch (state) {
            case Stage1:
                if (midTOF.getRange() > 150) {
                    m_shooter.kicker(0.5);
                    frontIntake.set(-IntakeSpeed);
                    backIntake.set(-IntakeSpeed);
                    m_shooter.shoot(-0.3);
                } else {
                    state = intakeState.Stage2;
                }
                break;
            case Stage2:
                if (midTOF.getRange() < 130) {
                    m_shooter.kicker(-0.2);
                } else {
                    state = intakeState.Stage3;
                }
                break;
            case Stage3:
                m_shooter.kicker(0);
                m_shooter.shoot(0);
                frontIntake.set(0);
                backIntake.set(0);
                break;
        }

    }

    public void resetIntakeState() {
        state = intakeState.Stage1;
    }

    public Command AutoIntake() {
        return this.runOnce(() -> Intake());
    }

    public void passthrough() {
        frontIntake.set(PassoverSpeed);
        backIntake.set(-PassoverSpeed);
    }

    public void stop() {
        frontIntake.set(0);
        backIntake.set(0);
    }
}
