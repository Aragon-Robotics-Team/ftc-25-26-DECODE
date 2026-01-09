package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;
import java.util.Comparator;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;

    // State tracking to avoid spamming the hardware bus
    private int currentPipeline = -1;

    // Tag Configurations
    private static final List<Integer> MOTIF_TAG_IDS = Arrays.asList(21, 22, 23); // Obelisk Tags
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24);      // Red/Blue Goals

    // Physics Constants
    private static final double MOUNTING_HEIGHT_IN = 8.45;
    private static final double MOUNTING_ANGLE_DEG = 0.0;
    private static final double ARTIFACT_HEIGHT_IN = 2.5;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure for high-speed tracking
        limelight.setPollRateHz(90);

        limelight.start();

        // Default to Intake/Ball Detection Mode on startup
        setPipeline(0);
    }

    /**
     * Switches the Limelight pipeline.
     * Includes a check to ensure we don't send redundant commands.
     * @param pipelineIndex The index to switch to 0-2.
     *                      0: Ball + Ramp Neural Detection
     *                      1: Ball Neural Detection
     *                      2: AprilTags
     */
    public void setPipeline(int pipelineIndex) {
        if (pipelineIndex != currentPipeline) {
            limelight.pipelineSwitch(pipelineIndex);
            currentPipeline = pipelineIndex;
        }
    }

    /**
     * Gets the raw result object from Limelight.
     */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    // -----------------------------------------------------------------------------------
    // AprilTag Logic (Requires Pipeline 1)
    // -----------------------------------------------------------------------------------

    /**
     * Identifies the primary "Motif" AprilTag visible.
     * Note: Returns null if current pipeline is not detecting tags.
     */
    public Integer detectMotif(LLResult result) {
        if (result == null || !result.isValid()) return null;

        return result.getFiducialResults().stream()
                .filter(fid -> MOTIF_TAG_IDS.contains(fid.getFiducialId()))
                .max(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetArea))
                .map(LLResultTypes.FiducialResult::getFiducialId)
                .orElse(null);
    }

    public Pose detectRobotPosition(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Ensure we aren't getting stale 0,0,0 data if pipeline just switched
                if (Math.abs(botpose.getPosition().x) < 0.01 && Math.abs(botpose.getPosition().y) < 0.01) {
                    return null;
                }

                double xInches = DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                double yInches = DistanceUnit.INCH.fromMeters(botpose.getPosition().y);
                return new Pose(xInches, yInches, 0);
            }
        }
        return null;
    }

    public Pose2D getFusionCorrection(double currentX, double currentY, double currentHeading) {
        // If we are in Intake mode (Pipeline 0), we cannot localize. Return null immediately.
        if (currentPipeline != 1) return null;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            if (botpose != null) {
                double visionX = DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                double visionY = DistanceUnit.INCH.fromMeters(botpose.getPosition().y);

                double drift = Math.hypot(currentX - visionX, currentY - visionY);

                if (drift > 2.0 && drift < 48.0) {
                    Pose2D correction =  new Pose2D(DistanceUnit.INCH, visionX, visionY, AngleUnit.RADIANS, currentHeading);
                    Pose correctionPedro = PoseConverter.pose2DToPose(correction, InvertedFTCCoordinates.INSTANCE);
                }
            }
        }
        return null;
    }

    // -----------------------------------------------------------------------------------
    // Artifact / Game Piece Detection (Requires Pipeline 0/2)
    // -----------------------------------------------------------------------------------

    public Double getArtifactHeading(LLResult result) {
        if (result != null && result.isValid()) {
            if (!result.getDetectorResults().isEmpty()) {
                return result.getDetectorResults().get(0).getTargetXDegrees();
            } else if (!result.getColorResults().isEmpty()) {
                return result.getColorResults().get(0).getTargetXDegrees();
            }
            // Standard tx is usually populated by the "best" target regardless of type
            return result.getTx();
        }
        return null;
    }

    public Double getArtifactDistance(LLResult result) {
        if (result != null && result.isValid()) {
            // Safety check: if we are in AprilTag mode, Ty might be a tag, not a ball.
            // But usually result lists are empty if the pipeline type doesn't match.
            double targetY = result.getTy();

            double heightDiff = ARTIFACT_HEIGHT_IN - MOUNTING_HEIGHT_IN;
            double angleRad = Math.toRadians(MOUNTING_ANGLE_DEG + targetY);

            return heightDiff / Math.tan(angleRad);
        }
        return null;
    }

    public Double detectGoalXDistance(LLResult result) {
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (GOAL_TAG_IDS.contains(fiducial.getFiducialId())) {
                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        return DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
                    }
                }
            }
        }
        return null;
    }
}