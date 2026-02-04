package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.geometry.Pose; // Import Pedro Pose

import java.util.Arrays;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private static final List<Integer> MOTIF_TAG_IDS = Arrays.asList(21, 22, 23);
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24);

    public enum LIMELIGHT_PIPELINES {
        APRILTAG,
        ARTIFACT_AND_RAMP,
        ARTIFACT_ONLY
    }

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(90);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setPipeline(LIMELIGHT_PIPELINES pipeline) {
        switch (pipeline) {
            case APRILTAG:
                limelight.pipelineSwitch(1);
                break;
            case ARTIFACT_AND_RAMP:
                limelight.pipelineSwitch(0);
                break;
            case ARTIFACT_ONLY:
                limelight.pipelineSwitch(2);
                break;
        }
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /**
     * Updates the Limelight with the robot's current heading.
     * REQUIRED for MegaTag2 to work.
     * @param headingRadians Robot heading in Radians (Pedro standard)
     */
    public void updateRobotOrientation(double headingRadians) {
        // Limelight expects degrees
        limelight.updateRobotOrientation(Math.toDegrees(headingRadians));
    }

    /**
     * Gets the robot's field position using MegaTag1.
     * @param result The latest LLResult
     * @return A PedroPathing Pose in INCHES, or null if invalid.
     */
    public Pose getMegaTag1Pose(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D botpose_mt1 = result.getBotpose();

            if (botpose_mt1 != null) {
                // 1. Convert Meters (Limelight) to Inches
                double xInches = botpose_mt1.getPosition().x * 39.3701;
                double yInches = botpose_mt1.getPosition().y * 39.3701;

                // 2. Create a Pose2D containing the converted units and the Yaw (Heading)
                // Note: We use the Vision Heading here because MT1 calculates it from the tag.
                Pose2D rawPose = new Pose2D(
                        DistanceUnit.INCH,
                        xInches,
                        yInches,
                        AngleUnit.DEGREES,
                        botpose_mt1.getOrientation().getYaw()
                );

                // 3. Convert Coordinate Systems (Limelight Center -> Pedro Corner)
                Pose ftcStandard = PoseConverter.pose2DToPose(rawPose, InvertedFTCCoordinates.INSTANCE);
                return ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            }
        }
        return null;
    }

    /**
     * Gets the robot's position and converts it to Pedro Pathing Coordinates.
     * @param result The latest LLResult
     * @return A valid Pose in Inches (Corner-Zero), or null if invalid.
     */
    public Pose getMegaTagGeminiPose(LLResult result) {
        // 1. Safety Checks (Paranoid Check to prevent Crashes)
        if (result == null || !result.isValid()) {
            return null;
        }

        // 2. Get the MT2 Botpose (or MT1 if you prefer)
        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            return null;
        }

        // 3. Extract Data (Limelight returns METERS from CENTER (0,0))
        double xMeters = botpose.getPosition().x;
        double yMeters = botpose.getPosition().y;
        double yawDegrees = botpose.getOrientation().getYaw();

        // 4. Convert to Inches
        double xInches = xMeters * 39.3701;
        double yInches = yMeters * 39.3701;

        // 5. Convert Coordinate System (Center -> Corner)
        // Pedro (0,0) is the corner. Center is (72, 72).
        // Note: You might need to swap X and Y depending on your camera mount rotation.
        // Assuming standard mounting (Camera Forward = Robot Forward):
        double pedroX = xInches + 72.0;
        double pedroY = yInches + 72.0;

        // 6. Return the Pose (Convert Yaw to Radians)
        return new Pose(pedroX, pedroY, Math.toRadians(yawDegrees));
    }

    public Integer detectMotif(LLResult result) {
        if (result != null && result.isValid()) {
            // Find the biggest tag (closest) to avoid noise
            LLResultTypes.FiducialResult biggestTag = null;
            double maxArea = 0;

            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (MOTIF_TAG_IDS.contains(id)) {
                    if (fiducial.getTargetArea() > maxArea) {
                        maxArea = fiducial.getTargetArea();
                        biggestTag = fiducial;
                    }
                }
            }
            if (biggestTag != null) return biggestTag.getFiducialId();
        }
        return null;
    }


    /**
     * @param result a detection from the limelight
     * Returns the horizontal distance of the center crosshair to the goal apriltags. Used for camera-only autoaim (not preferrable).
     * @return camera's horizontal distance from each specific tag's center as a double or null if nothing is found
     * */
    public Object detectGoalTy(LLResult result) {
        // Red ID = 24
        // Blue ID = 20
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (GOAL_TAG_IDS.contains(id)) {
                    return result.getTy();
                }
            }
        }
        return null;
    }

    /**
     * @param result limelight detection
     * @return megatag2 pose or null
     * */
    public Pose getMegaTag2Pose(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            if (botpose_mt2 != null) {
                Pose2D botpose_mt2_2d = new Pose2D(DistanceUnit.INCH, botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, AngleUnit.DEGREES, 0);

                Pose ftcStandard = PoseConverter.pose2DToPose(botpose_mt2_2d, InvertedFTCCoordinates.INSTANCE);
                Pose pedroStandard = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                return pedroStandard;
            }
        }
        return null;
    }

    //also claire remember u can use @param to annotate what you need to pass in
    /**
     * @return pass in list of detections to get the detected goal apriltag id or null if none is found
     * */
    public Object findAprilTag(LLResult result) {
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (GOAL_TAG_IDS.contains(id)) {
                    return id;
                }
            }
        }
        return null;
    }
    /**
     * @param result ll result
     * @return The angle that the closest (?) ball is from the camera, in degrees.
     */
    public Object findColorBlobHeading(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getTx(); //Apparently, tx is in degrees already.
        }
        return null;
    }

    /**
     * @param result ll result
     * @return The distance that the closest (?) ball is from the camera, in inches.
     */
    public Object findColorblobDistance(LLResult result) {
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double mountingHeight = 8.45; //The height of the camera lens from the floor (in)
            double mountingAngle = 0.0; //The mounting angle of the camera relative to the horizon (deg).
            double ballHeight = 5.0 / 2.0; //The height of the center of the ball (in)
            double distance = (ballHeight - mountingHeight) / Math.tan(Math.toRadians(ty + mountingAngle));
            return distance;
        }
        return null;
    }

    public void takeSnapshot() {
        limelight.captureSnapshot(String.valueOf(System.currentTimeMillis()));
    }
    public void takeSnapshot(String name) {
        limelight.captureSnapshot(name);
    }
}
