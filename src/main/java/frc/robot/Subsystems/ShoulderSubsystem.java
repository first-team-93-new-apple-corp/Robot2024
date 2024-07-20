package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    TalonFX ShoulderL;
    TalonFX ShoulderR;
    // CANcoder m_fakeCANcoder = new CANcoder(93);
    // CANcoderSimState m_CaNcoderSimState = new CANcoderSimState(m_fakeCANcoder);
    DutyCycleEncoder m_Encoder;
    TalonFXConfiguration m_configL;
    TalonFXConfiguration m_configR;
    // PositionVoltage m_PositionVoltage = new PositionVoltage(1);
    MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);
    MotionMagicConfigs m_MotionMagicConfigs = new MotionMagicConfigs();
    NeutralOut m_NeutralOut = new NeutralOut();
    double motorPos;
    double position;
    double setpoint;
    private double tempPos;

    // double kP = 1.00;
    // double kI = 0;
    // double kD = 0.4;
    // // double kG = 0.23; // 0.23
    // double kV = 5.39;
    // double kA = 0.01;
    // double kS = 0.1;
    public static final double ShoulderSpeed = 0.5;

    int periodicTimer = 0;

    public ShoulderSubsystem() {

        // Definitions
        m_Encoder = new DutyCycleEncoder(Constants.Sensors.DIO.ThroughBoreEncoder);

        ShoulderL = new TalonFX(Constants.CTRE.L_Shoulder);
        ShoulderR = new TalonFX(Constants.CTRE.R_Shoulder);

        // Config
        m_configL = new TalonFXConfiguration();
        m_configR = new TalonFXConfiguration();

        m_configL.CurrentLimits.StatorCurrentLimit = 20;
        m_configR.CurrentLimits.StatorCurrentLimit = 20;

        m_configL.CurrentLimits.StatorCurrentLimitEnable = true;
        m_configR.CurrentLimits.StatorCurrentLimitEnable = true;

        m_configL.Slot0.kP = 0.5;
        m_configL.Slot0.kI = 0;
        m_configL.Slot0.kD = 0;
        m_configL.Slot0.kV = 0.05; // 5.39
        m_configL.Slot0.kA = 0;
        m_configL.Slot0.kS = 0.175;

        // m_configL.Slot0.kG = 0.1;

        m_MotionMagicConfigs = m_configL.MotionMagic;
        m_MotionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
        m_MotionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
        // m_MotionMagicConfigs.MotionMagicJerk = 5;

        // Motor Setup
        ShoulderL.getConfigurator().apply(m_configL);
        ShoulderR.getConfigurator().apply(m_configR);

        ShoulderL.setInverted(false);
        ShoulderR.setInverted(false);

        ShoulderL.setNeutralMode(NeutralModeValue.Brake);
        ShoulderR.setNeutralMode(NeutralModeValue.Brake);

        // Set right motor to follow left to simplify
        ShoulderR.setControl(new Follower(ShoulderL.getDeviceID(), false));
        m_Encoder.setPositionOffset(0.5);

        /*
         * This in theory should allow us to reset the motor inbuilt encoder to the
         * value of the duty cycle encoder but
         * this is not the case, however, due to the play in chain and gearbox.
         * Resetting the motor encoder to the
         * duty cycle encoder value will cause the motor to move causes the system to
         * oscillate. Current idea is to create a "check" of sorts
         * to see if the motor encoder is within a certain range of the duty cycle
         * encoder and if it is not, reset the motor encoder to the
         * duty cycle encoder value. Also, resetting here doesn't seem to work, likely
         * due to the fact that we are applying motor configs at the same time, which
         * likely "locks"
         * the motor from receiving new values/inputs.
         */
        ShoulderL.setPosition(m_Encoder.get() * 300);
        ShoulderR.setPosition(m_Encoder.get() * 300);

    }

    public void toSetpoint(double setpointin) {
        setpoint = setpointin;
        ShoulderL.setControl(m_MotionMagicVoltage.withPosition(setpoint));
    }

    public void lock() {
        ShoulderL.set(0);
    }

    // Auto
    public void shoulderAuto() {
        ShoulderL.setControl(m_MotionMagicVoltage);
    }

    public double getPosition() {
        position = m_Encoder.get();
        return position;
    }

    public boolean atSetpoint() {
        SmartDashboard.putNumber("Setpoint", setpoint);

        return ShoulderL.getPosition().getValueAsDouble() < setpoint + 1
                && ShoulderL.getPosition().getValueAsDouble() > setpoint - 1;

    }

    public double getLeft() {
        return ShoulderL.getPosition().getValueAsDouble();
    }

    public double getRight() {
        return ShoulderR.getPosition().getValueAsDouble();
    }
    public void testup() {
        setpoint += 5;
        toSetpoint(setpoint);
    }
    public void testdown() {
        setpoint -= 5;
        toSetpoint(setpoint);
    }
    @Override
    public void periodic() {
        
        // Below is attempt #1 at figuring out the position issue but this method of
        // doing it caused lots of stuttering and oscillation
        // If I could do this 100s of times a second, it wouldn't be an issue (I have it set at 1 second for debugging purposes)

        // periodicTimer++;
        // if (periodicTimer > 50) {
        // ShoulderL.setPosition(m_Encoder.get() * 300);
        // ShoulderR.setPosition(m_Encoder.get() * 300);
        // periodicTimer = 0;
        // }
        // if (atSetpoint()) {
        // lock();
        // }

        //Below is attempt #2 (results pending)
        tempPos = getPosition() * 300;
        SmartDashboard.putNumber("tempPos",tempPos);
        // For this example, let's say that "acceptable" error in position is 1 rotation
        // of the motor in either direction (2 rotations)

        // I learned about Ternary operators (https://www.w3schools.com/java/java_conditions_shorthand.asp)
        // This isn't a great use case but it works for now
        // yes I know that I should just use a global variable but I'm lazy (it took longer to write this than to actually move it lol)
        boolean needsReset = (ShoulderL.getPosition().getValueAsDouble() < (tempPos - 4)
                || ShoulderL.getPosition().getValueAsDouble() > (tempPos + 4));
                SmartDashboard.putBoolean("needsReset", needsReset);
        if (needsReset) {
            // System.out.println("Reset shoulder motor position(s)");
            ShoulderL.setPosition(getPosition() * 300);
            ShoulderR.setPosition(getPosition() * 300);
        }
    }

}



