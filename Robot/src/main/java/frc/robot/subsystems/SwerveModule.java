package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    double StrafeSpeed;
    double motorcommand;
    double turningDegrees;
    AHRS Gyro;
    WPI_TalonFX DrivingMotor;
    WPI_TalonFX TurningMotor;
    WPI_CANCoder CanCoder;
    ProfiledPIDController TurningPID = new ProfiledPIDController(DriveConstants.TurningP, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14));
    PIDController DrivingPID = new PIDController(1, 0, 0);

    public SwerveModule(int driveMotorID, int turnMotorID, int CanCoderID, double magnetOffset) {
        DrivingMotor = new WPI_TalonFX(driveMotorID);
        TurningMotor = new WPI_TalonFX(turnMotorID);
        CanCoder = new WPI_CANCoder(CanCoderID);
        CanCoder.configMagnetOffset(magnetOffset);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(StrafeSpeed, calculateAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, calculateAngle());

        double driveOutput = state.speedMetersPerSecond / DriveConstants.maxStrafeSpeed * DriveConstants.MaxVolts; 
        double turnOutput = TurningPID.calculate(calculateAngle().getRadians(), state.angle.getRadians());
        // Calculate the turning motor output from the turning PID controller.
        DrivingMotor.setVoltage(driveOutput);
        TurningMotor.setVoltage(turnOutput);
    }

    public void resetEncoders() { //need to figure out offsets
        DrivingMotor.setSelectedSensorPosition(0);
        TurningMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        double speed = DrivingMotor.getSelectedSensorVelocity() * 10
                / DriveConstants.TalonFXEncoderResolution / DriveConstants.drivingGearing
                * DriveConstants.WheelCircumference;
        return Units.feetToMeters(speed);
    }

    public Rotation2d calculateAngle() { // get angle from cancoder
        return Rotation2d.fromDegrees(CanCoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}