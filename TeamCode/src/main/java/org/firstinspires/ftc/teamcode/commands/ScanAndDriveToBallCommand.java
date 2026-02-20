package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.function.BiFunction;
import java.util.function.Function;

public class ScanAndDriveToBallCommand extends CommandBase {
    Follower follower;
    LimelightSubsystem limelight;
    boolean finished = false;
    public ScanAndDriveToBallCommand(Follower follower, LimelightSubsystem limelight) {
        this.follower = follower;
        this.limelight = limelight;
    }
    public BiFunction<Double, Double, Pose> ballPoseSupplier = (distance, angle) -> {
        Pose currentPose = follower.getPose();

        double absoluteAngleRad = currentPose.getHeading() + Math.toRadians(angle);

        double targetX = currentPose.getX() + (Math.cos(absoluteAngleRad) * distance);
        double targetY = currentPose.getY() + (Math.sin(absoluteAngleRad) * distance);

        return new Pose(targetX, targetY);
    };
    public Function<Pose, PathChain> ballPathSupplier = pose -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, pose)))
            .setTangentHeadingInterpolation()
            .build();

    @Override
    public void initialize() {
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.ARTIFACT_ONLY);
        LLResult result = limelight.getResult();
        if (result.isValid()) {
            Pose ballPose = ballPoseSupplier.apply(limelight.findNearestBallDistance(result), -result.getTy());
            follower.followPath(ballPathSupplier.apply(ballPose));
        } else {
            finished = true;
        }
    }

    @Override
    public void execute() {
        if (!follower.isBusy() || follower.isRobotStuck()) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }
}
