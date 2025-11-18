package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.ShootBallSequenceCommandSequence;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;



@Config
@Autonomous(name = "Red Far 12ball real🦅", group = "angryBirds", preselectTeleOp = "Alpha Teleop")
public class RedFarAuto extends CommandOpMode {
    //paths
    private final ArrayList<PathChain> paths = new ArrayList<>();

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(88.5, 8.45, 90); //find actual statring pos
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    private RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{UNKNOWN,UNKNOWN,UNKNOWN};

    public void buildPaths(Follower follower) {
        follower.setStartingPose(startingPose);
        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.500, 8.450), new Pose(88.500, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70.12))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.500, 18.000), new Pose(101.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(70.12), Math.toRadians(0))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(101.000, 35.000), new Pose(131.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(131.000, 35.000), new Pose(88.500, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70.12))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.500, 18.000), new Pose(134.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(70.12), Math.toRadians(-30))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 20.000), new Pose(134.000, 10.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-25))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.500, 10.500), new Pose(88.500, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(70.12))
                .build());

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.500, 18.000), new Pose(88.500, 25.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(70.12), Math.toRadians(45))
                .build());
    }

    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(1500)
                ),
                new MoveSpindexerCommand(spindexer, gate, 1, false),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                ),
                new MoveSpindexerCommand(spindexer, gate, 1, false),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                ),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.REVERSE))
        );
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        lastVoltageCheck.reset();
        led = new LEDSubsystem(hardwareMap);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);

        //init paths
        buildPaths(follower);

        //schedule commands
        //one cycle = 3 balls + shoot given starting position is right at the shooting spot
        //pp file is editable but you have to update the buildPath

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setTargetVelocity(1300); //this is gonnna have to spin really fast for a realy long amount of time
                            shooter.setHood(0.56); //Placeholder
                            gate.up();
                            follower.setMaxPower(1);
                        }),

                        //starting shot from top of the spike
                        new FollowPathCommand(follower, paths.get(0), true), //drive to shooting pos
                        new InstantCommand(() -> spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE})), //MAKE SURE TO LOAD BALLS PROPERLY
                        new ShootBallSequenceCommandSequence(shooter, spindexer, gate, motif), //shoot motif

                        //cycle one
                        new FollowPathCommand(follower, paths.get(1), true), //drives to balls and lines itself up to intake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(0.7)), //slow intake
                                intakeArtifacts(),
                                new ParallelRaceGroup(
                                        new FollowPathCommand(follower, paths.get(2), true), //drive and pick up balls
//                                        new WaitForRobotStuckCommand(follower) //does not work for some reason
                                        new WaitCommand(3000)
                                )

                        ),
                        new InstantCommand(() -> follower.setMaxPower(1)), //return to normal speed

                        new FollowPathCommand(follower, paths.get(3), true), // returning to shooting pos
                        new InstantCommand(() -> spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, PURPLE, GREEN})), //last row
                        new ShootBallSequenceCommandSequence(shooter, spindexer, gate, motif), //shoot motif

                        //cycle two
                        new FollowPathCommand(follower, paths.get(4), true), //drive to corner to corner balls and intake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(0.7)), //might want to figure this speed out to optimize corner intake
                                intakeArtifacts(),
                                new ParallelRaceGroup(
                                        new FollowPathCommand(follower, paths.get(5), true), //drive and pick up balls
//                                        new WaitForRobotStuckCommand(follower) //does not work for some reason
                                        new WaitCommand(3000)
                                )

                        ),
                        new InstantCommand(() -> follower.setMaxPower(1)), //return to normal speed

                        new FollowPathCommand(follower, paths.get(6), true), // returning to shooting pos
                        //this may not be best to be hardcoded since intaking in the corner is kind of variable
                        new InstantCommand(() -> spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, GREEN, PURPLE})), //corner
                        new ShootBallSequenceCommandSequence(shooter, spindexer, gate, motif), //shoot motif

                        //move off corner spike and turn off shooter
                        new FollowPathCommand(follower, paths.get(7)),
                        new InstantCommand(() -> {shooter.setTargetVelocity(0);})
                )
        );


    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
            led.setColor(LEDSubsystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubsystem.LEDState.GREEN);
        }
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }


        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("is spindexer ready to read color ", spindexer.availableToSenseColor());
        telemetry.addData("spindexer's balls", spindexer.getBalls());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());

        telemetry.addData("------------------",null);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",null);

        timer.reset();
        telemetry.update();
        super.run();

    }
}
