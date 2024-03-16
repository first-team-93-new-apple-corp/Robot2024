// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Intake.IO;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeIOSim implements IntakeIO {
    private final double PassoverSpeed;
    private final double IntakeSpeed;
    private TalonFX frontIntake;
    private TalonFX backIntake;
    private TalonFX bumperIntake;
    private TalonFXConfiguration config;
    private TimeOfFlight midTOF;
    private TimeOfFlight upperTOF;
    private LEDSubsystem m_LED;
    private ShooterSubsystem m_shooter;
    private XboxController op;
    private TalonFXSimState frontIntakeSimState;
    private TalonFXSimState backIntakeSimState;
    private TalonFXSimState BumperIntakeSimState;

    public enum intakeState {
        Stage1,
        Stage2,
        Stage3
    }

    private intakeState state = intakeState.Stage1;


    public IntakeIOSim(IntakeConstants constants, LEDSubsystem m_LED, ShooterSubsystem m_shooter, XboxController op) {
        this.m_LED = m_LED;
        this.m_shooter = m_shooter;
        this.op = op;
        PassoverSpeed = IntakeConstants.PassoverSpeed;
        IntakeSpeed = IntakeConstants.IntakeSpeed;
        frontIntake = new TalonFX(constants.F_Intake, "rio");
        backIntake = new TalonFX(constants.B_Intake, "rio");
        bumperIntake = new TalonFX(constants.Bump_Intake, "drivetrain");
        midTOF = new TimeOfFlight(constants.midTOF);
        upperTOF = new TimeOfFlight(constants.upperTOF);
        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        frontIntake.getConfigurator().apply(config);
        backIntake.getConfigurator().apply(config);
        bumperIntake.getConfigurator().apply(config);
        backIntake.setInverted(false);
        bumperIntake.setInverted(true);
        midTOF.setRangingMode(RangingMode.Short, 24);
        upperTOF.setRangingMode(RangingMode.Short, 24);
        frontIntakeSimState = frontIntake.getSimState(); 
        backIntakeSimState = backIntake.getSimState();
        BumperIntakeSimState = bumperIntake.getSimState();
    }

    @Override
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

    @Override
    public void updateValues(IntakeIOInputs inputs) {
        if (state == intakeState.Stage3) {
            inputs.NoteInIntake = true;
        } else {
            inputs.NoteInIntake = false;
        }
    }


    @Override
    public void resetIntakeState() {
        state = intakeState.Stage1;
    }


    @Override
    public void stop() {
        frontIntake.set(0);
        backIntake.set(0);
        bumperIntake.set(0);
    }



    @Override
    public void passthrough() {
        frontIntake.set(PassoverSpeed);
        backIntake.set(-PassoverSpeed);
        bumperIntake.set(PassoverSpeed);
    }

}
