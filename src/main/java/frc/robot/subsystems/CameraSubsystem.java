// package frc.robot.subsystems;

// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CameraSubsystem extends SubsystemBase {
//   UsbCamera camera; 
//   CvSink cvSink;
//   CvSource outputStream; 
//   Mat RawMat;
//   public CameraSubsystem() {
//      camera = CameraServer.startAutomaticCapture();
//             camera.setResolution(640, 480);
//             cvSink = CameraServer.getVideo();
//             outputStream = CameraServer.putVideo("Rectangle", 640, 480);
//             RawMat = new Mat();
//             // Mat EdgeMat = new Mat();
//   }
//   public void FlipCamera() {
//     Core.flip(RawMat, RawMat, 90);
          
//   }
//   public void periodic() {
//       if (cvSink.grabFrame(RawMat) == 0) {
//         outputStream.notifyError(cvSink.getError());
//         return;
//       }
//       outputStream.putFrame(RawMat); // Remember to change this to EDGEMAT when looking for Edges
//   }
// }