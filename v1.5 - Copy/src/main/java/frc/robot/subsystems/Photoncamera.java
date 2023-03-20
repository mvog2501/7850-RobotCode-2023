package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photoncamera {

    
    

    //7850Camera
    PhotonCamera camera = new PhotonCamera ("Team7850Camera");

    //Gets the latest result from the camera
    var result = camera.getLatestResult();

    //check for new targets
    boolean hasTargets = result.hasTargets();

    //gets list of current targets
    List<PhotonTrackedTarget> targets = result.getTargets();

    //gets current best target
    PhotonTrackedTarget target = result.getBestTarget();\

    //gets target information
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    Transform2d pose = target.getCameraToTarget();
    List<TargetCorner> corners = target.getCorners();

    //Getting more information from the target
    int targetID - target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // Capture pre-process camera stream image
    camera.takeInputSnapshot();

    // Capture post-process camera stream image
    camera.takeOutputSnapshot();

    if (xboxController.getAButton()) {
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


}
