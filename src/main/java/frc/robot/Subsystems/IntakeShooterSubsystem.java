package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class IntakeShooterSubsystem extends SubsystemBase {
    TalonFX Shooter = new TalonFX(Constants.CTRE.Shoot);
    TalonFX Intake = new TalonFX(Constants.CTRE.Intake);

    TalonFXConfiguration m_configIntake;
    TalonFXConfiguration m_configShooter;

    TalonFX m_motorIntake;
    TalonFX m_motorShooter;

    PositionVoltage m_PositionVoltage = new PositionVoltage(0);

    // XboxController op;

    TimeOfFlight TOF;

    final double MuzzleIntake = -0.30;
    final double AmpShooterSpeed = 0.1;
    final double KickerSpeed = 1;
    private double IntakeSpeed = 0.25;

    boolean notePastTOF = false;
    double SpeakerShooterSpeed = 0.55;
    double StartTime;
    double currentspeed;

    public void init() {
        // this.op = op;

        m_configIntake = new TalonFXConfiguration();
        m_configShooter = new TalonFXConfiguration();

        m_motorIntake = new TalonFX(Constants.CTRE.Intake);
        m_motorShooter = new TalonFX(Constants.CTRE.Shoot);

        m_configIntake.CurrentLimits.SupplyCurrentLimit = 25;
        m_configIntake.CurrentLimits.SupplyCurrentThreshold = 25;
        m_configIntake.CurrentLimits.SupplyTimeThreshold = 0;
        m_configIntake.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_configIntake.Slot0.kP = 0.05;
        m_configIntake.Slot0.kI = 0;
        m_configIntake.Slot0.kD = 0;

        m_configShooter.Slot0.kP = 0.05;
        m_configShooter.Slot0.kI = 0;
        m_configShooter.Slot0.kD = 0;

        m_motorIntake.getConfigurator().apply(m_configIntake);
        m_motorShooter.getConfigurator().apply(m_configShooter);
        m_motorIntake.setInverted(true);

        TOF = new TimeOfFlight(22);
        TOF.setRangingMode(RangingMode.Short, 24);
    }

    // **************************************************************Intake*******************************************************

    public enum intakeState {
        IntakeStage1,
        IntakeStage2,
        BreakStage
    }

    private intakeState state = intakeState.IntakeStage1;

    public void Intake() {
        switch (state) {
            case IntakeStage1:
                Intake.set(IntakeSpeed);
                Shooter.set(0.05);
                // op.setRumble(RumbleType.kBothRumble, 0.5);
                if (TOF.getRange() < 100) {
                    notePastTOF = true;
                } else if ((TOF.getRange() > 100) && (notePastTOF == true)) {
                    state = intakeState.IntakeStage2;
                }
                break;
            case IntakeStage2:
                if (notePastTOF == true && TOF.getRange() > 100) {
                    Intake.set(-0.1);
                    Shooter.set(-0.1);
                    // op.setRumble(RumbleType.kBothRumble, 0.5);
                } else {
                    state = intakeState.BreakStage;
                }
                break;
            case BreakStage:
                if (TOF.getRange() > 100) {
                    state = intakeState.IntakeStage1;
                } else {
                    notePastTOF = false;
                    Shooter.set(0);
                    Intake.set(0);
                    // op.setRumble(RumbleType.kBothRumble, 0);
                }
                break;
        }
    }

    public void resetIntakeState() {
        state = intakeState.IntakeStage1;
    }

    public Command stop() {
        return this.runOnce(() -> Intake.set(0));
    }

    public Command AutoIntake() {
        return this.runOnce(() -> Intake());
    }

    public Command AutonStopIntake() {
        return this.runOnce(() -> stop());
    }

    public void intakePeriodic() {
        SmartDashboard.putBoolean("Note In Intake?", notePastTOF);
    }

    // **************************************************************Shooter*******************************************************

    public void shoot(double speed) {
        Shooter.set(speed);
    }

    public void prime() {
        Shooter.set(SpeakerShooterSpeed);
    }

    public void AutonPrime() {
        Shooter.set(.60);
    }

    public void shootAmp() {
        Shooter.set(AmpShooterSpeed);
    }

    public void kicker(double KickerSpeed) {
        Intake.set(KickerSpeed);
    }

    public void intakeFront() {
        Shooter.set(MuzzleIntake);
    }

    public void shooterStop() {
        Shooter.set(0);
    }

    public void kickerStop() {
        Intake.set(0);
    }

    public void ShootingforAuton() {
        AutonPrime();
        kicker(KickerSpeed);
    }

    public Command AutonShooter() {
        return this.runOnce(() -> ShootingforAuton());
    }

    public Command AutonStopShooter() {
        return this.runOnce(() -> shooterStop());
    }

    public void increaseSpeed() {
        if (SpeakerShooterSpeed <= 0.95) {
            SpeakerShooterSpeed += 0.05; // +5%speed
        }
    }

    public void decreaseSpeed() {
        if (SpeakerShooterSpeed >= 0.1) {
            SpeakerShooterSpeed -= 0.05; // -5%speed
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putNumber("Shooter Speed", SpeakerShooterSpeed);
        SmartDashboard.putNumber("Shooter RPM", Shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Temp", Shooter.getDeviceTemp().getValueAsDouble());
    }

    // **************************************************************Misc.*******************************************************
    @Override
    public void periodic() {
        shooterPeriodic();
        intakePeriodic();
    }
}