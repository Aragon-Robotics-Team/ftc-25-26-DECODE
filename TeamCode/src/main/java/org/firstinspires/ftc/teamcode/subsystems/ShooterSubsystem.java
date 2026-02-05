package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;
import com.seattlesolvers.solverslib.util.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private Motor shooter1;
    private Motor shooter2;
    private MotorGroup shooter;
    public double getVelocityTicks() {
        return shooter1.getCorrectedVelocity();
    }
    public boolean isAtTargetVelocity() {
        return Math.abs(flywheelController.getSetPoint() - shooter1.getCorrectedVelocity()) < 50;
    }
    double kPOriginal = -0.012; //although the coefficients are negative it is fine because it works;
    double kFOriginal = -0.00052;
    double kP = kPOriginal;
    double kF = kFOriginal;
    InterpLUT lut;
    double speedMax;
    double speedMin;
    double distMax;
    double distMin;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);
    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.BARE);

        shooter1.setInverted(true); //one has to be backwards
        shooter2.setInverted(false);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        
        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0);

        //Note: The distance measured is from the robot center to the spot where the ball lands in the corner, NOT the apriltag.
        lut = new InterpLUT(); //distance (in), linear speed (in/s);
//        lut.add(46.5, 435);
//        lut.add(60, 450);
//        lut.add(80, 490);
//        lut.add(206, 540);
//        lut.add(232.5, 570);

//        lut.add(59,470);
//        lut.add(66, 466);
//        lut.add(82,500);
//        lut.add(122,585);
//        lut.add(132,590);
//        lut.add(134,610);
//        lut.add(141,620);
        lut.add(60.0,474.0);
        lut.add(65.0,474.0);
        lut.add(70.0,474.0);
        lut.add(75.0,483.0);
        lut.add(80.0,483.0);
        lut.add(85.0,517.0);
        lut.add(90.0,517.0);
        lut.add(100.0,525.0);
        lut.add(135.0,576.0);
        lut.add(142.0,593.0);
        lut.add(150.0,602.0);
        lut.add(157.0,650.0);
        lut.createLUT();
        speedMax = 650.0;
        speedMin = 474.0;
        distMax = 157.0;
        distMin = 60.0;
    }
    public double getSpeedMax() {
        return speedMax;
    }
    public double getSpeedMin() {
        return speedMin;
    }
    public double getDistMax() {
        return distMax;
    }
    public double getDistMin() {
        return distMin;
    }
    public void setPIDF(double p, double i, double d, double f) {
        this.kPOriginal = p;
        this.kFOriginal = f;
        this.flywheelController.setPIDF(p, i, d, f);

        // Temporarily apply directly to current kP/kF in case voltage comp isn't running
        this.kP = p;
        this.kF = f;
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
        return -shooter1.getCorrectedVelocity() / ticksPerRev * gearRatio * Math.PI * flywheelDiameter;
    }
    public double findSpeedFromDistance(double distance) {
        double clampedDistance = MathUtils.clamp(distance, distMin, distMax);
        Double result =  lut.get(clampedDistance);
        return (result != null) ? result : speedMin;
    }
    public void updatePIDVoltage(double voltage) {
//        double compensation = 13.5 / voltage; //if voltage < 13.5, compensation > 1
        double compensation = 1;
        kP = compensation * kPOriginal;
        kF = compensation * kFOriginal;
    }
    @Override
    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        shooter.set(flywheelController.calculate(flywheelController.getSetPoint() != 0 ? shooter1.getCorrectedVelocity() : 0));
    }

}
