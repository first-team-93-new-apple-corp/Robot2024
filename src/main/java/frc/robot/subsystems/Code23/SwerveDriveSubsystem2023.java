package frc.robot.subsystems.Code23;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import frc.robot.subsystems.DriveBaseSubsystem;

public class SwerveDriveSubsystem2023 extends DriveBaseSubsystem {


    public SwerveDriveSubsystem2023(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveDriveSubsystem2023(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void startSimThread() {
        // TODO Auto-generated method stub

    }

    @Override
    public void configAuto() {
        // TODO Auto-generated method stub

    }
}
