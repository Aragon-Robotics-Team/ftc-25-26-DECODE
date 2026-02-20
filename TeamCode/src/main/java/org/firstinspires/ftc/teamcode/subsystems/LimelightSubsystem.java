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
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {
    public Limelight3A limelight;
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
     * Gets the robot's field position using MegaTag2.
     * @param result The latest LLResult
     * @return A PedroPathing Pose in INCHES, or null if invalid.
     */
    public Pose getMegaTagPose(LLResult result) {
        if (result != null && result.isValid()) {
            // MegaTag2 is robust because it uses our IMU heading
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            if (botpose_mt2 != null) {
                //converting to 2D
                //im 98 percent sure getYaw returns degrees by default so you have to specify
                Pose2D mt1Conversion2D = new Pose2D(DistanceUnit.METER,botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, AngleUnit.DEGREES, botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES));

                //discord method
                Pose ftcStandard = PoseConverter.pose2DToPose(mt1Conversion2D, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                return pedroPose;
            }
        }
        return null; //might want to replcae with robots last known pose as a failsafe
    }

    public Pose getFTCSTANDARD(LLResult result) {
        if (result != null && result.isValid()) {
            // MegaTag2 is robust because it uses our IMU heading
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            if (botpose_mt2 != null) {
                //converting to 2D
                //im 98 percent sure getYaw returns degrees by default so you have to specify
                Pose2D mt1Conversion2D = new Pose2D(DistanceUnit.METER,botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, AngleUnit.RADIANS, botpose_mt2.getOrientation().getYaw(AngleUnit.RADIANS));

                //discord method
                Pose ftcStandard = PoseConverter.pose2DToPose(mt1Conversion2D, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                return ftcStandard;
            }
        }
        return null; //might want to replcae with robots last known pose as a failsafe

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
