// package frc.robot.subsystems;

// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CameraSubsystem extends SubsystemBase {
//     UsbCamera camera;
//     CvSink cvSink;
//     CvSource outputStream, outputStreamOrange;
//     Mat RawMat;
//     Mat hsv;
//     double LB = 193, LG = 246, LR = 255, HB = 57, HG = 84, HR = 251;
//     Scalar lowerOrange = new Scalar(LR, LG, LB);
//     Scalar higherOrange = new Scalar(HR, HG, HB);

//     public CameraSubsystem() {
//         camera = CameraServer.startAutomaticCapture();
//         camera.setResolution(640, 480);
//         cvSink = CameraServer.getVideo();
//         outputStream = CameraServer.putVideo("Rectangle", 640, 480);
//         outputStreamOrange = CameraServer.putVideo("OrangeRectangle", 640, 480);
//         RawMat = new Mat();
//         hsv = RawMat.clone();
//         // Mat EdgeMat = new Mat();
//     }

//     public void FlipCamera() {
//         Core.flip(RawMat, RawMat, 90);

//     }

//     public void periodic() {
//         if (cvSink.grabFrame(RawMat) == 0) { // getting frame
//             outputStream.notifyError(cvSink.getError());
//             return;
//         }
//         // lowerOrange = new Scalar(LR, LG, LB);
//         // higherOrange = new Scalar(HR, HG, HB);
//         Imgproc.cvtColor(RawMat, hsv, Imgproc.COLOR_BGR2HSV); // Convert color to hsv
//         Core.inRange(hsv, higherOrange, lowerOrange, hsv); // Filtering image based on color range
//         outputStreamOrange.putFrame(hsv);
//         outputStream.putFrame(RawMat); // Remember to change this to EDGEMAT when looking for Edges
//         // SmartDashboard.putNumber("LB", LB);
//         // SmartDashboard.putNumber("LG", LG);
//         // SmartDashboard.putNumber("LR", LR);
//         // SmartDashboard.putNumber("HB", HB);
//         // SmartDashboard.putNumber("HG", HG);
//         // SmartDashboard.putNumber("HR", HR);
//         // LB  = SmartDashboard.getNumber("LB", LB);
//         // LG  = SmartDashboard.getNumber("LG", LG);
//         // LR  = SmartDashboard.getNumber("LR", LR);
//         // HB  = SmartDashboard.getNumber("HB", HB);
//         // HG  = SmartDashboard.getNumber("HG", HG);
//         // HR  = SmartDashboard.getNumber("HR", HR);
//     }
// }