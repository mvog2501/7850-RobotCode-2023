package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class Photoncamera {

    
    

    //7850Camera
    PhotonCamera camera = new PhotonCamera ("Team7850Camera");

    public void PhotoncameraTest() {
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        // Check if the latest result has any targets.
        boolean hasTargets = result.hasTargets();

        //gets list of current targets
        List<PhotonTrackedTarget> targets = result.getTargets();

        //gets current best target
        PhotonTrackedTarget target = result.getBestTarget();

        //gets target information
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        Transform2d pose = target.getCameraToTarget();
        List<TargetCorner> corners = target.getDetectedCorners();

        //Getting more information from the target
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        // Capture pre-process camera stream image
        camera.takeInputSnapshot();

        // Capture post-process camera stream image
        camera.takeOutputSnapshot();

        if (Joystick.getRawButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
        }

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
            }

                double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);

        // Calculate a translation from the camera to the target.
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
        distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
        
        // Calculate robot's field relative pose
        Pose2D robotPose = PhotonUtils.estimateFieldToRobot(
        kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);

        // Calculate robot's field relative pose
        Pose3D robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);

        Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);

        // The parameter for loadFromResource() will be different depending on the game.
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        //Forward Camera
        cam = new PhotonCamera("testCamera");
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update();
        }
    }


}
