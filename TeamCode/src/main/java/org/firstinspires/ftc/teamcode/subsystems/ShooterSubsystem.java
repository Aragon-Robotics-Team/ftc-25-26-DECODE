package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ShooterSubsystem extends SubsystemBase {
    private MotorEx shooter1;
    private MotorEx shooter2;
    private MotorGroup shooter;
    private ServoEx hood;
    public double getVelocityTicks() {
        return -shooter2.getCorrectedVelocity();
    }
    public boolean isAtTargetVelocity() {
        return Math.abs(flywheelController.getSetPoint() - shooter1.getCorrectedVelocity()) < 50;
    }
    public double kPOriginal = -0.0080; //although the coefficients are negative it is fine because it works;
    public double kFOriginal = -0.00052;
    double kP = kPOriginal;
    double kF = kFOriginal;
    InterpLUT distance_v_speed;
    InterpLUT angle_v_hoodPos;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);
    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new MotorEx(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new MotorEx(hMap, "shooter2", Motor.GoBILDA.BARE);
        hood = new ServoEx(hMap, "hood");

        shooter1.setInverted(true); //one has to be backwards
        shooter2.setInverted(false);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        
        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0);

        //Note: The distance measured is from the robot center to the spot where the ball lands in the corner, NOT the apriltag.
        distance_v_speed = new InterpLUT(); //distance (in), linear speed (in/s);
        distance_v_speed.add(60.0,474.0);
        distance_v_speed.add(65.0,474.0);
        distance_v_speed.add(70.0,474.0);
        distance_v_speed.add(75.0,483.0);
        distance_v_speed.add(80.0,483.0);
        distance_v_speed.add(85.0,517.0);
        distance_v_speed.add(90.0,517.0);
        distance_v_speed.add(100.0,525.0);
        distance_v_speed.add(135.0,576.0);
        distance_v_speed.add(142.0,593.0);
        distance_v_speed.add(150.0,602.0);
        distance_v_speed.add(157.0,650.0);
        distance_v_speed.createLUT();

        angle_v_hoodPos = new InterpLUT(); //launch angle v. hood servo pos.
        angle_v_hoodPos.add(45,0.01);
        angle_v_hoodPos.add(90,1);
        angle_v_hoodPos.createLUT();
    }
    public void setPIDF(double p, double i, double d, double f) {
        this.kPOriginal = p;
        this.kFOriginal = f;
        this.flywheelController.setPIDF(p, i, d, f);

        // Temporarily apply directly to current kP/kF in case voltage comp isn't running
        this.kP = p;
        this.kF = f;
    }
    public void setHoodPos(double pos) {
        hood.set(pos);
    }

    /**
     * Sets a raw target in Ticks Per Second (bypasses linear math).
     * Useful for initial feedforward tuning.
     */
    public void setTargetTicks(double ticksPerSec) {
        flywheelController.setSetPoint(ticksPerSec);
    }

    /**
     * @return The target velocity in Ticks Per Second
     */
    public double getTargetTicks() {
        return flywheelController.getSetPoint();
    }
    public void setTargetLinearSpeed(double vel) {
        double ticksPerRev = 28.0;
        double flywheelDiameter = 2.83465;
        double gearRatio = 32.0 / 24.0; //
        double flywheelRPS = vel / (Math.PI * flywheelDiameter);
        double motorRPS = flywheelRPS / gearRatio;
        double targetTicksPerSec = motorRPS * ticksPerRev;
        flywheelController.setSetPoint(targetTicksPerSec);
    }

    /**
     * hardware call on encoder
     * @return Linear speed of flywheel in in/s
     */
    public double getFlywheelLinearSpeed() {
        double ticksPerRev = 28.0;
        double flywheelDiameter = 2.83465; //72 mm to inches
        double gearRatio = 32.0 / 24.0;
        return shooter2.getCorrectedVelocity() / ticksPerRev * gearRatio * Math.PI * flywheelDiameter;
    }
    public double findSpeedFromDistance(double distance) {
        return distance_v_speed.get(distance);
    }
    public void updateHoodPosition(double flywheelLinearSpeed, double distance) {
        final double DISTANCE_OFFSET = 2; //2 inches
        final double GOAL_HEIGHT = 39;
        final double LAUNCH_HEIGHT = 10;
        final double GRAVITY = 386.22; //in/s^2
        final double MIN_ANGLE = 30;
        final double MAX_ANGLE = 75;

        double targetDistance = distance + DISTANCE_OFFSET; //Aim slightly deep to clear the lip
        double deltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
        double v = flywheelLinearSpeed;
        double g = GRAVITY;

        //Define coefficients for quadratic: A*tan²(θ) - x*tan(θ) + (A + Δy) = 0
        //Where A = (g * x²) / (2 * v²)
        double A = (g * Math.pow(targetDistance, 2)) / (2 * Math.pow(v, 2));

        double a_quad = A;
        double b_quad = -targetDistance;
        double c_quad = A + deltaY;

        //Solve Quadratic Formula: tan(θ) = (-b ± √(b² - 4ac)) / 2a
        double discriminant = Math.pow(b_quad, 2) - 4 * a_quad * c_quad;

        if (discriminant >= 0) {
            // We have valid solutions.
            // root1 corresponds to the higher arc (lob).
            // root2 corresponds to the lower arc (direct shot).
            double tanTheta1 = (-b_quad + Math.sqrt(discriminant)) / (2 * a_quad);
            double tanTheta2 = (-b_quad - Math.sqrt(discriminant)) / (2 * a_quad);

            double angle1 = Math.toDegrees(Math.atan(tanTheta1));
            double angle2 = Math.toDegrees(Math.atan(tanTheta2));

            // Select the best angle.
            // Usually, the lower angle (angle2) is preferred for faster travel time,
            // but we must ensure it is physically possible for the hood (e.g., > 0).
            // If the shot is impossible (too far/slow), we might need the higher arc.
            double optimalAngle = angle2;

            // Fallback to high arc if low arc is invalid (negative or too flat to clear obstacles)
            // However, with deltaY > 0, angle2 should always be positive.

            // Clamp angle to physical hood limits
            optimalAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, optimalAngle));

            hood.set(angle_v_hoodPos.get(optimalAngle));
        } else {
            // No solution exists (speed is too low for this distance).
            // Default to a known "safe" angle (like 45) or max power position.
            hood.set(angle_v_hoodPos.get(45));
        }
    }

    public void updatePIDVoltage(double voltage) {
//        double compensation = 13.5 / voltage; //if voltage < 13.5, compensation > 1
        double compensation = 1;
        kP = compensation * kPOriginal;
        kF = compensation * kFOriginal;
    }

    public double getShooter1CurrentAmps() {
        return shooter1.getCurrent(CurrentUnit.AMPS);
    }

    public double getShooter2CurrentAmps() {
        return shooter2.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        shooter.set(flywheelController.calculate(flywheelController.getSetPoint() != 0 ? -shooter2.getCorrectedVelocity() : 0));
    }

}
