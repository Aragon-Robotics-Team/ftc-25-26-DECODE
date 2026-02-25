package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import java.util.List;
@TeleOp(name = "Limelight Functions Test")
public class RampBallTest extends CommandOpMode {
    private ElapsedTime timer;
    private LimelightSubsystem limelight;
    @Override
    public void initialize() {
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.ARTIFACT_AND_RAMP);
        timer = new ElapsedTime();
        timer.reset();
    }
    @Override
    public void run() {
        LLResult result = limelight.getResult();

        if (result == null || result.getDetectorResults().isEmpty()) {
            telemetry.addData("Status", "No detections");
            telemetry.update();
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        List<List<Double>> rampCorners = null;
        for (LLResultTypes.DetectorResult detection : detections) {
            if (detection.getClassName().equals("ramp")) {
                rampCorners = detection.getTargetCorners();
                break;
            }
        }

        int artifactInRampCount = 0;

        for (LLResultTypes.DetectorResult detection : detections) {
            String objectName = detection.getClassName();
            double confidence = detection.getConfidence();
            List<List<Double>> artifactCorners = detection.getTargetCorners();
            if (artifactCorners != null && !artifactCorners.isEmpty()) {
                double[] center = calculateCenter(artifactCorners);

                // If we found a ramp, check if this center is inside it
                if (rampCorners != null) {
                    if (isPointInsideBox(center, rampCorners)) {
                        artifactInRampCount++;
                    }
                }
            }
        }

        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.addData("Debug: LLResult object", result);
        telemetry.addData("Num Balls in Ramp: ", artifactInRampCount);
    }

    /**
     * Calculates the center of a list of coordinates by averaging X and Y.
     * Returns a double array where index 0 is X and index 1 is Y.
     */
    private double[] calculateCenter(List<List<Double>> corners) {
        double sumX = 0;
        double sumY = 0;

        for (List<Double> point : corners) {
            sumX += point.get(0);
            sumY += point.get(1);
        }

        double centerX = sumX / corners.size();
        double centerY = sumY / corners.size();

        return new double[]{centerX, centerY};
    }
    
    /**
     * Checks if a point is inside a bounding box
     */
    private boolean isPointInsideBox(double[] point, List<List<Double>> boxCorners) {
        // Ensure we have exactly 4 corners for this vector math to work
        if (boxCorners == null || boxCorners.size() != 4) {
            return false;
        }

        double px = point[0]; // Point M (X)
        double py = point[1]; // Point M (Y)

        // Point A (Corner 0)
        double ax = boxCorners.get(0).get(0);
        double ay = boxCorners.get(0).get(1);

        // Point B (Corner 1)
        double bx = boxCorners.get(1).get(0);
        double by = boxCorners.get(1).get(1);

        // Point D (Corner 3) - Skip Corner 2 (Point C) as we only need the two adjacent sides
        double dx = boxCorners.get(3).get(0);
        double dy = boxCorners.get(3).get(1);

        // Vector AM (from A to our artifact center)
        double amX = px - ax;
        double amY = py - ay;

        // Vector AB (from A to B)
        double abX = bx - ax;
        double abY = by - ay;

        // Vector AD (from A to D)
        double adX = dx - ax;
        double adY = dy - ay;

        // Calculate Dot Products
        double amDotAb = (amX * abX) + (amY * abY);
        double abDotAb = (abX * abX) + (abY * abY);

        double amDotAd = (amX * adX) + (amY * adY);
        double adDotAd = (adX * adX) + (adY * adY);

        // The point is inside if it falls within the bounds of both vectors
        boolean withinAB = (0 <= amDotAb) && (amDotAb <= abDotAb);
        boolean withinAD = (0 <= amDotAd) && (amDotAd <= adDotAd);

        return withinAB && withinAD;
    }
}
