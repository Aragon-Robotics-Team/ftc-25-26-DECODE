package org.firstinspires.ftc.teamcode.kalman;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;

public class KalmanFilterMatrix {
    private Matrix X; // State vector 3x1: [x, y, heading]^T
    private Matrix P; // Covariance matrix 3x3

    private final Matrix Q; // Process noise (Odometry Trust)
    private final Matrix R; // Measurement noise (Vision Trust)
    private final Matrix I; // Identity matrix 3x3

    public KalmanFilterMatrix(double pinpointTrust, double limelightTrust) {
        // Q: How much we trust our odometry delta (smaller = trust more)
        // We set the 3rd diagonal (heading) extremely low because Pinpoint IMUs are near-perfect.
        Q = MatrixUtil.diagonal3(pinpointTrust, pinpointTrust, 0.001);

        // R: How much we trust our vision measurements (larger = trust less)
        // We set the 3rd diagonal (heading) extremely high so the filter ignores Limelight's heading.
        R = MatrixUtil.diagonal3(limelightTrust, limelightTrust, 999.0);

        I = MatrixUtil.identity(3);
        X = new Matrix(3, 1);
        P = MatrixUtil.identity(3);
    }

    public void reset(Pose startPose) {
        X.setMatrix(new double[][]{
                {startPose.getX()},
                {startPose.getY()},
                {startPose.getHeading()}
        });
        P = MatrixUtil.identity(3);
    }

    /**
     * Step 1: Predict where the robot is based on odometry wheels
     */
    public void predict(double deltaX, double deltaY, double deltaHeading) {
        Matrix U = new Matrix(new double[][]{
                {deltaX},
                {deltaY},
                {deltaHeading}
        });

        // X = X + U
        X = X.plus(U);

        // P = P + Q
        P = P.plus(Q);
    }

    /**
     * Step 2: Update the position based on the absolute Limelight coordinate
     */
    public void update(Pose visionPose) {
        if (visionPose == null) return; // Skip update if no AprilTags are visible

        Matrix Z = new Matrix(new double[][]{
                {visionPose.getX()},
                {visionPose.getY()},
                {visionPose.getHeading()}
        });

        // 1. Innovation: Y = Z - X  (Difference between vision and our prediction)
        Matrix Y = Z.minus(X);

        // 2. Innovation Covariance: S = P + R
        Matrix S = P.plus(R);

        // 3. Kalman Gain: K = P * S^-1  (Using Pedro's fast 3x3 inverter!)
        Matrix K = P.multiply(MatrixUtil.invert3x3(S));

        // 4. State Update: X = X + (K * Y)
        X = X.plus(K.multiply(Y));

        // 5. Covariance Update: P = (I - K) * P
        P = I.minus(K).multiply(P);
    }

    /**
     * Extracts the 3x1 State Matrix back into a PedroPathing Pose
     */
    public Pose getState() {
        return new Pose(X.get(0, 0), X.get(1, 0), X.get(2, 0));
    }
}