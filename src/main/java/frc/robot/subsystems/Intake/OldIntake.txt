package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.ElevatorSubsystem.elevatorState;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX frontIntake;
    private TalonFX backIntake;
    private TalonFX bumperIntake;
    TalonFXConfiguration config;
    TimeOfFlight midTOF;
    TimeOfFlight upperTOF;
    private double IntakeSpeed = 0.75;
    private double PassoverSpeed = 0.5;
    private ShooterSubsystem m_shooter;
    private LEDSubsystem m_LED;
    XboxController op;
    boolean noteInIntake;

    public enum intakeState {
        Stage1,
        Stage2,
        Stage3
    }

    private intakeState state = intakeState.Stage1;

    public IntakeSubsystem( ShooterSubsystem m_shooter, XboxController op, LEDSubsystem m_LED) {
        this.m_shooter = m_shooter;
        this.op = op;
        this.m_LED = m_LED;
        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        frontIntake = new TalonFX(Constants.CTRE.RIO.F_Intake, "rio");
        backIntake = new TalonFX(Constants.CTRE.RIO.B_Intake, "rio");
        bumperIntake = new TalonFX(Constants.CTRE.RIO.Bump_Intake, "drivetrain");
        frontIntake.getConfigurator().apply(config);
        backIntake.getConfigurator().apply(config);
        bumperIntake.getConfigurator().apply(config);
        backIntake.setInverted(false);
        midTOF = new TimeOfFlight(22);
        midTOF.setRangingMode(RangingMode.Short, 24);
        upperTOF = new TimeOfFlight(25);
        upperTOF.setRangingMode(RangingMode.Short, 24);
        bumperIntake.setInverted(true);
    }

    public Command AutonStopIntake() {
        return this.runOnce(() -> stop());
    }

    public void Intake() {
        switch (state) {
            case Stage1:
                if (midTOF.getRange() > 150) {
                    m_LED.noteAlmostInBot();
                    m_shooter.kicker(0.5);
                    frontIntake.set(-IntakeSpeed);
                    backIntake.set(-IntakeSpeed);
                    bumperIntake.set(-IntakeSpeed * 1.27);
                    m_shooter.shoot(-0.3);
                    op.setRumble(RumbleType.kBothRumble, 0.5);
                } else {
                    state = intakeState.Stage2;
                    m_LED.noteAlmostInBot();
                }
                break;
            case Stage2:
                if (midTOF.getRange() < 130) {
                    m_LED.noteAlmostInBot();
                    m_shooter.kicker(-0.1);
                    op.setRumble(RumbleType.kBothRumble, 0.5);
                } else {
                    state = intakeState.Stage3;
                    m_LED.noteInBot();
                }
                break;
            case Stage3:
                if (upperTOF.getRange() < 130) {
                    state = intakeState.Stage1;
                    m_LED.noteAlmostInBot();
                } else {
                    m_LED.noteInBot();
                    m_shooter.kicker(0);
                    m_shooter.shoot(0);
                    frontIntake.set(0);
                    backIntake.set(0);
                    bumperIntake.set(0);
                    op.setRumble(RumbleType.kBothRumble, 0);
                }
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
        bumperIntake.set(PassoverSpeed);
    }

    public void stop() {
        frontIntake.set(0);
        backIntake.set(0);
        bumperIntake.set(0);
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Note In Intake?", noteInIntake);
    }
}
