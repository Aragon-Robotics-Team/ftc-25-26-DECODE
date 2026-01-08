package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;
import java.util.Comparator;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;

    // Tag Configurations
    private static final List<Integer> MOTIF_TAG_IDS = Arrays.asList(21, 22, 23); // Obelisk Tags
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24);      // Red/Blue Goals

    // Physics Constants
    private static final double MOUNTING_HEIGHT_IN = 8.45;
    private static final double MOUNTING_ANGLE_DEG = 0.0;
    private static final double ARTIFACT_HEIGHT_IN = 2.5; // Approx height of sample on floor

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure for high-speed tracking
        limelight.setPollRateHz(90);

        // We use a SINGLE pipeline (0) configured as a "Detector"
        // in Limelight OS. This allows it to detect Neural Network Artifacts
        // AND AprilTags simultaneously.
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Gets the raw result object from Limelight.
     * Useful if other commands need to extract specific data types.
     */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /**
     * Identifies the primary "Motif" AprilTag visible.
     * Logic: Returns the ID of the motif tag with the LARGEST area (closest to robot).
     * * @param result The latest Limelight result.
     * @return The Integer ID of the closest motif tag, or null if none found.
     */
    public Integer detectMotif(LLResult result) {
        if (result == null || !result.isValid()) return null;

        // Filter for Motif tags and find the one with the max Area (ta)
        return result.getFiducialResults().stream()
                .filter(fid -> MOTIF_TAG_IDS.contains(fid.getFiducialId()))
                .max(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetArea))
                .map(LLResultTypes.FiducialResult::getFiducialId)
                .orElse(null);
    }

    /**
     * Gets the robot's current field position based on AprilTag MegaTag1.
     * Converts Limelight Meters to Pedro Pathing Inches.
     * * @param result The latest Limelight result.
     * @return A Pedro Pose (x, y) in inches, or null if localization failed.
     */
    public Pose detectRobotPosition(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose(); // MT1 is often more stable for pure FTC localization if MT2 isn't set up
            if (botpose != null) {
                double xInches = DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                double yInches = DistanceUnit.INCH.fromMeters(botpose.getPosition().y);
                // We typically ignore Vision Heading for localization as Pinpoint is more accurate
                return new Pose(xInches, yInches, 0);
            }
        }
        return null;
    }

    /**
     * Fusion Logic: Checks vision data against Pinpoint data.
     * If the vision data is high quality and within a reasonable "drift" window,
     * it returns a correction Pose to reset the Pinpoint.
     *
     * @param currentX Current Pinpoint X in Inches.
     * @param currentY Current Pinpoint Y in Inches.
     * @param currentHeading Current Pinpoint Heading (Radians).
     * @return A Pose to reset the Pinpoint to, or null if no correction is needed.
     */
    public Pose getFusionCorrection(double currentX, double currentY, double currentHeading) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            // Ensure we have a valid solve
            if (botpose != null) {
                double visionX = DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                double visionY = DistanceUnit.INCH.fromMeters(botpose.getPosition().y);

                // Calculate Euclidean drift distance
                double drift = Math.hypot(currentX - visionX, currentY - visionY);

                // Thresholding:
                // 1. Minimum 2 inch drift (don't jitter the odo for tiny noise)
                // 2. Maximum 48 inch drift (sanity check - if it says we are 4 feet away, vision is likely glitching/jumping)
                if (drift > 2.0 && drift < 48.0) {
                    // Return the Vision X/Y but KEEP the Pinpoint Heading (IMU is superior to Vision for rotation)
                    return new Pose(visionX, visionY, currentHeading);
                }
            }
        }
        return null;
    }

    // -----------------------------------------------------------------------------------
    // Artifact / Game Piece Detection
    // -----------------------------------------------------------------------------------

    /**
     * Returns the heading (tx) to the primary detector result (Artifact/Sample).
     * Used for aligning the intake to a sample.
     */
    public Double getArtifactHeading(LLResult result) {
        if (result != null && result.isValid()) {
            // Priority: Check Neural Detector Results first (Model), then Color Results
            if (!result.getDetectorResults().isEmpty()) {
                return result.getDetectorResults().get(0).getTargetXDegrees();
            } else if (!result.getColorResults().isEmpty()) {
                return result.getColorResults().get(0).getTargetXDegrees();
            }
            // If main pipeline result (tx) is valid
            return result.getTx();
        }
        return null;
    }

    /**
     * Calculates horizontal distance to the artifact using trigonometry.
     * @return Distance in inches.
     */
    public Double getArtifactDistance(LLResult result) {
        if (result != null && result.isValid()) {
            double targetY = result.getTy(); // Vertical offset in degrees

            // Trig: d = (h_target - h_camera) / tan(mount_angle + y_angle)
            double heightDiff = ARTIFACT_HEIGHT_IN - MOUNTING_HEIGHT_IN;
            double angleRad = Math.toRadians(MOUNTING_ANGLE_DEG + targetY);

            return heightDiff / Math.tan(angleRad);
        }
        return null;
    }

    // -----------------------------------------------------------------------------------
    // Legacy / specific helper methods
    // -----------------------------------------------------------------------------------

    /**
     * Find the horizontal distance to a goal apriltag (for auto-aim).
     */
    public Double detectGoalXDistance(LLResult result) {
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (GOAL_TAG_IDS.contains(fiducial.getFiducialId())) {
                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        // Return X distance in Inches
                        return DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                    }
                }
            }
        }
        return null;
    }
}