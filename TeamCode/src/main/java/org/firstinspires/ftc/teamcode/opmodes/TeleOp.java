package org.firstinspires.ftc.teamcode.opmodes;

import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.SHOOTER_ANGLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;
import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop Field Centric", group = "!")
public class TeleOp extends CommandOpMode {
    //Constants
    private ElapsedTime snapshotTimer;
    public enum Alliance {
        RED,
        BLUE
    }
    public enum IntakeState {
        INTAKESTILL_ROLLERSIN, REVERSE, INTAKING, REVERSE_AND_INTAKE_ROLLERS
    }
    int speedMin;
    int speedMax;
    int distMin;
    int distMax;
    int closeShooterTarget;
    int farShooterTarget;
    boolean isAdjustingFar = false;
    boolean isHoldingPoint = false;
    int snapshots = 0;
    double headingError;
    private Pose holdPose = new Pose(); // Tracks where we want to stay
    final Pose GOAL_RED = new Pose(135,141.5);
    final Pose GOAL_BLUE = new Pose(9,141.5);
    final RobotConstants.BallColors[] PPG = {RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN};
    final RobotConstants.BallColors[] GPP = {RobotConstants.BallColors.GREEN, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE};
    final RobotConstants.BallColors[] PGP = {RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN, RobotConstants.BallColors.PURPLE};
    final RobotConstants.BallColors[] XXX = {RobotConstants.BallColors.UNKNOWN, RobotConstants.BallColors.UNKNOWN, RobotConstants.BallColors.UNKNOWN};

    // 2. Create a master list of all cycleable options
    final RobotConstants.BallColors[][] allMotifs = {PPG, GPP, PGP};
    int index = 0;

    //State variables
    Alliance alliance = Alliance.RED;
    RobotConstants.BallColors[] selectedMotif = new RobotConstants.BallColors[]{RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN};
    IntakeState intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
    boolean manualControl = true;
    boolean slowMode = false;

    //subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private LEDSubsystem led;
    private GateSubsystem gate;
    private ClimbSubsystem climb;
    private LimelightSubsystem limelight;
    public VoltageSensor voltageSensor;
    public GamepadEx driver1;
    public GamepadEx driver2;

    //Pedro and PID
    private Follower follower;
    public static Pose startingPose;
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;
    //Auto aligner
    public static double alignerHeadingkP = -0.01;
    public static double alignerHeadingkD = 0.0;
    public static double alignerHeadingkF = 0.0;
    PIDFController alignerHeadingPID = new PIDFController(alignerHeadingkP, 0, alignerHeadingkD, alignerHeadingkF);
    double lastSeenX;
    double headingVector;

    //Voltage compensation
    double currentVoltage = 14;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();

    //Timer
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    @Override
    public void initialize () {
        initializeSystems();
        snapshotTimer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        createBinds();
        speedMax = shooter.getSpeedMax();
        speedMin = shooter.getSpeedMin();
        distMax = shooter.getDistMax();
        distMin = shooter.getDistMin();
        closeShooterTarget = 505; //450;
        farShooterTarget = 660; //540;
        snapshotTimer.reset();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        handleTeleopDrive();
        handleLED();
        handleVoltageCompensation();
        handleBallsArrayUpdate();

        //Update color sensors
        colorSensors.updateSensor1();
        colorSensors.updateSensor2();

        handleTelemetry();

        follower.update();
        loopTimer.reset();
        telemetry.update();
        super.run();

        if (snapshotTimer.seconds() > 5) {
            snapshots++;
            limelight.takeSnapshot();
            snapshotTimer.reset();
        }
    }
    void initializeSystems() {
        startingPose = (Pose) blackboard.get("endpose");
        if (startingPose == null) {
            startingPose = new Pose(104,135.8,Math.toRadians(-90));
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorSensors = new ColorSensorsSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.APRILTAG);
        climb = new ClimbSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        spindexer.set(115);//75
        gate.down();

        super.reset();
        lastVoltageCheck.reset();
        register(intake, shooter, spindexer, gate, colorSensors, led, limelight);
        follower.startTeleopDrive();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
    }
    void createBinds() {
        //Driver 1
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.INTAKING) intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
                    else intakeState = IntakeState.INTAKING;
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.REVERSE_AND_INTAKE_ROLLERS) intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
                    if (intakeState == IntakeState.REVERSE) intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
                    else intakeState = IntakeState.REVERSE;
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false)
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new ParallelCommandGroup(
                    new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, -1, true, false),
                    new InstantCommand(() -> {
                        intakeState = IntakeState.REVERSE_AND_INTAKE_ROLLERS;
                        new SelectCommand(this::getIntakeCommand).schedule();
                    })
                )
        );
        new Trigger( //Auto aim
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> {
                            manualControl = false;
                        })
                );
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //slowmode
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));

        //Driver 2
        driver2.getGamepadButton(DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    selectedMotif = XXX;
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> {
                    index++;
                    index%=3;
                    selectedMotif = allMotifs[index];
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    index++;
                    index+=3;
                    index%=3;
                    selectedMotif = allMotifs[index];
                })
        );
        driver2.getGamepadButton(LEFT_BUMPER).whenActive(  //turn off shooter
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(0);
                    gamepad2.rumbleBlips(1);
                    selectedMotif = allMotifs[index];
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand(() -> {
                    climb.climbUp();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    climb.climbDown();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    follower.setPose(follower.getPose().setHeading(0));
                    gamepad2.rumbleBlips(1);
                }));
        driver2.getGamepadButton(RIGHT_BUMPER).whenActive(  //shooter close
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(closeShooterTarget);
                })
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //shooter far
                .whileActiveContinuous(new InstantCommand(() -> {
                            shooter.setTargetLinearSpeed(farShooterTarget);
                        })
                );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //shooter far
                .whileActiveContinuous(new InstantCommand(() -> {
                            shooter.setTargetLinearSpeed(farShooterTarget);
                        })
                );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //intake
                .whenActive(new InstantCommand(() -> {
                            isAdjustingFar = true;
                        })
                )
                .whenInactive(new InstantCommand(() -> {
                            isAdjustingFar = false;
                        })
                );
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (isAdjustingFar) {
                        farShooterTarget += 10;
                        gamepad2.rumbleBlips(1);
                    } else{
                        closeShooterTarget += 10;
                        gamepad2.rumbleBlips(1);
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (isAdjustingFar) {
                        farShooterTarget -= 10;
                        gamepad2.rumbleBlips(1);
                    } else{
                        closeShooterTarget -= 10;
                        gamepad2.rumbleBlips(1);
                    }
                })
        );

    }
    void handleTeleopDrive() {
        LLResult result = limelight.getResult();

        //Drivetrain code
        if (manualControl) {
            //shooter.setTargetLinearSpeed(50);
            double x = driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = -driver1.getRightX() * (slowMode ? 0.3 : 1.2);
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            double magnitude = Math.abs(x) + Math.abs(y) + Math.abs(rx);
//            if (magnitude> 0.1) {
//                holdPose = follower.getPose();
//                if (isHoldingPoint && follower.getVelocity().getMagnitude() < 4) { //in/s, random nubmer
//                    follower.startTeleopDrive();
//                    isHoldingPoint = false;
//                }
//
//            } else {
//                if (!isHoldingPoint) {
//                    holdPose = follower.getPose(); //occurs on falling edge of holding point
//                    isHoldingPoint = true;
//                    follower.holdPoint(holdPose);
//                }
//            }
            follower.setTeleOpDrive(x / denominator, y / denominator, rx / denominator, false);
        } else {
            // --- AUTO AIM MODE ---
            if (gamepad1.touchpad_finger_1) {
                manualControl = true;
                gamepad1.rumbleBlips(1);
            }

            double x = driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = -driver1.getRightX() * (slowMode ? 0.3 : 1.2);
            headingError = 0;
            if (result != null && result.isValid()) {
                headingError = result.getTy() + 1;
            }


            rx += MathUtils.clamp(alignerHeadingPID.calculate(headingError, 0), -0.5, 0.5);

            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(x / denominator, y / denominator, rx / denominator, false);
        }
    }
    void handleLED() {
        //LED Code
        if (intakeState == IntakeState.INTAKESTILL_ROLLERSIN) {
            led.setColor(LEDSubsystem.LEDState.WHITE);
        }
        else if (intakeState == IntakeState.INTAKING) {
            led.setColor(LEDSubsystem.LEDState.YELLOW);
        }
        else if (shooter.getVelocityTicks() > 300) { //shooting mode
            if (shooter.getVelocityTicks() - shooter.getTargetTicks() < -30) {
                led.setColor(LEDSubsystem.LEDState.RED);
            }
            else if (shooter.getVelocityTicks() - shooter.getTargetTicks() > 50) {
                led.setColor(LEDSubsystem.LEDState.BLUE);
            }
            else {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
        }
        else {
            double t = totalTimer.seconds();
            double min = 0.3;
            double max = 0.722;
            double amplitude = (max - min) / 2.0;
            double midpoint = (max + min) / 2.0;
            double speed = 0.6; // cycles per second — increase for faster transitions

            // Oscillate servo position smoothly with sine wave
            double position = midpoint + amplitude * Math.sin(2 * Math.PI * speed * t);
            led.setPosition(position);
        }
    }
    void handleVoltageCompensation() {
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }
    }
    void handleBallsArrayUpdate() {
        //spindexer and array logic
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorSensors.getIntakeSensor1Result(), colorSensors.getIntakeSensor2Result(), colorSensors.getBackResult());
        }
    }
    void handleTelemetry() {
        telemetry.addLine(alliance == Alliance.RED ? "\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34" : "\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35");
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("headingError", headingError);
        telemetry.addData("Mode", manualControl ? "Manual" : "Auto-Aim");
        telemetry.addData("Selected Motif", Arrays.toString(selectedMotif));
        telemetry.addData("Balls Array", Arrays.toString(spindexer.getBalls()));

        telemetry.addLine("--Spindexer--");
        telemetry.addData("PID output", spindexer.getOutput());
        telemetry.addData("PID setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("Unwrapped position", spindexer.getCurrentPosition());
        telemetry.addLine("--Shooter--");
        telemetry.addData("Target ticks", shooter.getTargetTicks());
        telemetry.addData("Actual ticks ", shooter.getVelocityTicks());
        telemetry.addData("Linear speed ", shooter.getFlywheelLinearSpeed());
        telemetry.addLine("--Color Sensors--");
        NormalizedRGBA val1 = colorSensors.getIntakeSensor1Result();
        NormalizedRGBA val2 = colorSensors.getIntakeSensor2Result();
        NormalizedRGBA valBack = colorSensors.getBackResult();
        // -- Sensor 1 (Intake) --
        String color1 = "None";
        float[] hsv1 = {0,0,0};
        if (val1 != null) {
            hsv1 = ColorSensorsSubsystem.rgbToHsv(val1);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val1)) color1 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val1)) color1 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val1)) color1 = "White";
        }
        telemetry.addData("Intake 1", "[%s] H:%.0f S:%.2f V:%.2f", color1, hsv1[0], hsv1[1], hsv1[2]);
        // -- Sensor 2 (Intake) --
        String color2 = "None";
        float[] hsv2 = {0,0,0};
        if (val2 != null) {
            hsv2 = ColorSensorsSubsystem.rgbToHsv(val2);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val2)) color2 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val2)) color2 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val2)) color2 = "White";
        }
        telemetry.addData("Intake 2", "[%s] H:%.0f S:%.2f V:%.2f", color2, hsv2[0], hsv2[1], hsv2[2]);
        // -- Back Sensor (Uses Back-specific logic) --
        String colorBack = "None";
        float[] hsvBack = {0,0,0};
        if (valBack != null) {
            hsvBack = ColorSensorsSubsystem.rgbToHsv(valBack);
            if (ColorSensorsSubsystem.colorIsPurpleBack(valBack)) colorBack = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenBack(valBack)) colorBack = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(valBack)) colorBack = "White";
        }
        telemetry.addData("Back", "[%s] H:%.0f S:%.2f V:%.2f", colorBack, hsvBack[0], hsvBack[1], hsvBack[2]);
        telemetry.addLine("--Pedro--");
        telemetry.addData("Position ", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("Heading ", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("Slow mode", slowMode);
        telemetry.addData("Blackboard endpose", (Pose) blackboard.get("endpose"));
        telemetry.addData("snapshots taken", snapshots);

    }
    /**
     * Calculate the target vector for the shooter with velocity compensation.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector(Follower follower, Pose targetPose, ShooterSubsystem shooter) {
        Pose robotPose = follower.getPose();
        Vector v_robot = follower.getVelocity(); //Assume its inches per second. in polar
        Pose robotFuturePose = robotPose.plus(new Pose(v_robot.getXComponent(), v_robot.getYComponent()).times(0.200)); //200 ms latency for shooter
        double dx = targetPose.getX() - robotFuturePose.getX();
        double dy = targetPose.getY() - robotFuturePose.getY();
        double speed = shooter.findSpeedFromDistance(Math.hypot(dx, dy));
        double horizontalSpeed = speed * Math.cos(SHOOTER_ANGLE);
        double idealHeading = Math.atan2(dy, dx);
        Vector v_target = new Vector(speed, idealHeading); //is polar
        Vector v_ball = v_target.minus(v_robot);
        return new Vector(v_ball.getMagnitude(), v_ball.getTheta() / Math.cos(SHOOTER_ANGLE));
    }

    /**
     * Calculate the target vector for the ball with velocity and acceleration compensation, as well as latency in the shooter.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector2(Follower follower,Pose robotPose, Pose targetPose, ShooterSubsystem shooter) {
        // --- 0. CONFIGURATION ---
        // You must estimate your shooter's launch angle relative to the floor.
        // If your hood moves, calculate this based on hood position.
        // For fixed hoods, 45-60 degrees is common.
        double launchAngle = SHOOTER_ANGLE;
        double latency = 0.015+0.16;

        // --- 1. GATHER CURRENT STATE ---
        Pose currentPose = robotPose;
        Vector v_robot = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();
        Vector a_robot = follower.getAcceleration();

        // Offsets (Distance in inches from center of robot to shooter)
        double shooterOffsetX = 5.0;
        double shooterOffsetY = 0.0;

        // --- 2. PREDICT ROBOT POSE (Standard Kinematics) ---
        double futureHeading = currentPose.getHeading() + (angularVel * latency);

        // Position Prediction
        double predX = currentPose.getX() + (v_robot.getXComponent() * latency) + (0.5 * a_robot.getXComponent() * latency * latency);
        double predY = currentPose.getY() + (v_robot.getYComponent() * latency) + (0.5 * a_robot.getYComponent() * latency * latency);

        // --- 3. CALCULATE ROBOT VELOCITY AT MUZZLE (Standard Rigid Body) ---
        double futureVx = v_robot.getXComponent() + (a_robot.getXComponent() * latency);
        double futureVy = v_robot.getYComponent() + (a_robot.getYComponent() * latency);

        double cosH = Math.cos(futureHeading);
        double sinH = Math.sin(futureHeading);

        // Rotated offset
        double fieldOffsetX = (shooterOffsetX * cosH) - (shooterOffsetY * sinH);
        double fieldOffsetY = (shooterOffsetX * sinH) + (shooterOffsetY * cosH);

        // Tangential velocity
        double v_tangential_x = -angularVel * fieldOffsetY;
        double v_tangential_y =  angularVel * fieldOffsetX;

        // Total Robot Velocity Components (Cartesian)
        double finalRobotVx = futureVx + v_tangential_x;
        double finalRobotVy = futureVy + v_tangential_y;

        // Convert to Polar for Pedro Vector
        double robotVelMag = Math.hypot(finalRobotVx, finalRobotVy);
        double robotVelAngle = Math.atan2(finalRobotVy, finalRobotVx);
        Vector v_robot_total = new Vector(robotVelMag, robotVelAngle);

        // --- 4. SOLVE FOR SHOOTING VECTOR ---
        double shooterMsgX = predX + fieldOffsetX;
        double shooterMsgY = predY + fieldOffsetY;

        double dx = targetPose.getX() - shooterMsgX;
        double dy = targetPose.getY() - shooterMsgY;
        double dist = Math.hypot(dx, dy);
        double idealHeading = Math.atan2(dy, dx);

        // === THE FIX STARTS HERE ===

        // A. Get the Total Exit Speed required for this distance (from your lookup table/regression)
        double clampedDist = clamp(dist, distMin+1, distMax-1);
        double totalSpeedRequired = shooter.findSpeedFromDistance(clampedDist);

        // B. "Flatten" this speed to the 2D floor plane
        //    We only want the horizontal component for vector math
        double horizontalSpeed = totalSpeedRequired * Math.cos(launchAngle);

        // C. Create the target vector using Horizontal Speed
        Vector v_target_horizontal = new Vector(horizontalSpeed, idealHeading);

        // D. Perform Vector Subtraction in the 2D plane
        //    (Horizontal Target) - (Horizontal Robot Motion) = (Horizontal Ball Launch Vector)
        Vector v_ball_horizontal = v_target_horizontal.minus(v_robot_total);

        // E. Convert the result back to Total Exit Speed for the flywheel
        //    Total = Horizontal / cos(theta)
        double finalHorizontalSpeed = v_ball_horizontal.getMagnitude();
        double finalTotalSpeed = finalHorizontalSpeed / Math.cos(launchAngle);

        // Return a Vector with the NEW Total Speed and the CORRECTED heading
        return new Vector(finalTotalSpeed, v_ball_horizontal.getTheta());
    }
    /**
     * Calculates the smallest difference between two angles in radians.
     * @return a double between -PI and +PI.
     */
    public double getAngleDifference(double targetAngle, double currentAngle) {
        double difference = targetAngle - currentAngle;

        // Normalize the angle to be within -PI to +PI
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference <= -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }
    public Command getIntakeCommand() {
        switch (intakeState) {
            case INTAKING:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSOUT);
                });
            case REVERSE:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN);
                });
            case REVERSE_AND_INTAKE_ROLLERS:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSIN);
                });
            case INTAKESTILL_ROLLERSIN:
            default:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKESTILL_ROLLERSIN);
                });
        }
    }
}