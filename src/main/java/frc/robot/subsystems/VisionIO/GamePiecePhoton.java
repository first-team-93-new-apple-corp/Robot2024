package frc.robot.subsystems.VisionIO;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePiecePhoton extends SubsystemBase {
    // Pigeon2 pigeon2;
    private double rotation;
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    List<PhotonPipelineResult> results;
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    double pitch, yaw, area;

    private PIDController rotate = new PIDController(0.04, 0, 0.001);

    public GamePiecePhoton() {
        rotate.setSetpoint(0);
        rotate.setTolerance(2, 0.2);
    }

    public double turnToNote() {
        return rotate.calculate(yaw);
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                target = result.getBestTarget();

                yaw = target.getYaw();
                pitch = target.getPitch();
                area = target.getArea();
            }
        }  
        SmartDashboard.putBoolean("CameraConnected?", camera.isConnected());
        SmartDashboard.putNumber("Photon rotation", rotation);
        SmartDashboard.putNumber("Photon yaw", yaw);
    }
}
