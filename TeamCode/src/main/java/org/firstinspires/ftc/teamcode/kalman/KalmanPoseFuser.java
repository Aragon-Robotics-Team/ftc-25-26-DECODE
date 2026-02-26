package org.firstinspires.ftc.teamcode.kalman;

import com.pedropathing.geometry.Pose;

public class KalmanPoseFuser {
    private KalmanFilterMatrix filter;
    private Pose previousPinpointPose;

    /**
     * @param startPose The robot's starting position on the field
     * @param pinpointTrust Trust in odometry delta (e.g., 0.01)
     * @param limelightTrust Trust in absolute vision position (e.g., 0.5)
     */
    public KalmanPoseFuser(Pose startPose, double pinpointTrust, double limelightTrust) {
        filter = new KalmanFilterMatrix(pinpointTrust, limelightTrust);
        filter.reset(startPose);
        this.previousPinpointPose = startPose;
    }

    /**
     * Call this every loop to get your fused, smoothed Pose.
     * @param currentPinpointPose Raw pose from PedroPathing/Pinpoint
     * @param limelightPose Absolute pose from Limelight (pass null if no tags visible)
     * @return The fused Pose
     */
    public Pose update(Pose currentPinpointPose, Pose limelightPose) {
        // Calculate the change in odometry since the last loop
        double deltaX = currentPinpointPose.getX() - previousPinpointPose.getX();
        double deltaY = currentPinpointPose.getY() - previousPinpointPose.getY();
        double deltaHeading = currentPinpointPose.getHeading() - previousPinpointPose.getHeading();

        // Step 1: Push odometry deltas into the filter
        filter.predict(deltaX, deltaY, deltaHeading);

        // Step 2: Push vision data into the filter (if available)
        filter.update(limelightPose);

        // Save Pinpoint pose for the next loop's delta math
        previousPinpointPose = currentPinpointPose;

        // Return the smoothed Pose!
        return filter.getState();
    }

    /**
     * Force the fuser to a specific coordinate (useful for Auto init or manual resets).
     */
    public void setPose(Pose pose) {
        filter.reset(pose);
        previousPinpointPose = pose;
    }
}