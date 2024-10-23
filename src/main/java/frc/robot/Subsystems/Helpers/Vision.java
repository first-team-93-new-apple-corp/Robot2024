package frc.robot.subsystems.Helpers;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("limelight-front")
            .getEntry("botpose_targetspace");
    Pigeon2 pigeon2;
    private Double[] values;
    private Double[] defaultValues;
    private double rotation;
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    double pitch;
    double yaw;
    double area;
    private PIDController rotate = new PIDController(0.1, 0, 0);

    public Vision(Pigeon2 pigeon2) {

        this.pigeon2 = pigeon2;
        defaultValues = new Double[6];
        defaultValues[0] = 0.;
        defaultValues[1] = 0.;
        defaultValues[2] = 0.;
        defaultValues[3] = 0.;
        defaultValues[4] = 0.;
        defaultValues[5] = 0.;
    }

    public double turnToNote() {
        return rotate.calculate(yaw, 0);
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            
            yaw = target.getYaw();
            pitch = target.getPitch();
            area = target.getArea();
        }
        values = limelight.getDoubleArray(defaultValues);
        rotation = values[4];
        SmartDashboard.putNumberArray("limelight values", values);
        SmartDashboard.putNumber("RotatePid", turnToNote());
        SmartDashboard.putNumber("yaw", yaw);
    }
    public boolean cameraHasNote(){
        return result.hasTargets();
    }
}
