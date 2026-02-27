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
    boolean avoidFar;
    boolean isFollowingPath = false;
    long startTime;

    public ScanAndDriveToBallCommand(Follower follower, LimelightSubsystem limelight, boolean avoidFar) {
        this.follower = follower;
        this.limelight = limelight;
        this.avoidFar = avoidFar;
    }

    public BiFunction<Double, Double, Pose> ballPoseSupplier = (distance, angle) -> {
        Pose currentPose = follower.getPose();
        double safeDistance = Math.min(distance, 62);

        double absoluteAngleRad = currentPose.getHeading() + Math.toRadians(angle);

        double targetX = currentPose.getX() + (Math.cos(absoluteAngleRad) * safeDistance);
        double targetY = currentPose.getY() + (Math.sin(absoluteAngleRad) * safeDistance);

        return new Pose(targetX, targetY);
    };

    public Function<Pose, PathChain> ballPathSupplier = pose -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower.getPose(), pose)))
            .setTangentHeadingInterpolation()
            .build();

    @Override
    public void initialize() {
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.ARTIFACT_ONLY);
        startTime = System.currentTimeMillis();
        isFollowingPath = false;
        finished = false;
        follower.setMaxPower(0.6);
    }

    @Override
    public void execute() {
        if (!isFollowingPath) {
            LLResult result = limelight.getResult();

            if (result != null && result.isValid() && result.getTa() > 0.5) {
                Double rawDistance = limelight.findNearestBallDistance(result);

                if (rawDistance != null && rawDistance > 0) {
                    Pose ballPose = ballPoseSupplier.apply(rawDistance + 17, result.getTy());
                    boolean crossesCenterLine = (follower.getPose().getX() - 72) * (ballPose.getX() - 72) < 0;

                    if (avoidFar && ballPose.getY() > 60 && !crossesCenterLine) {
                        finished = true;
                    } else {
                        follower.followPath(ballPathSupplier.apply(ballPose));
                        isFollowingPath = true;
                    }
                }
            }
            else if (System.currentTimeMillis() - startTime > 500) {
                finished = true;
            }
        }
        else {
            if (!follower.isBusy() || follower.isRobotStuck()) {
                finished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
        follower.setMaxPower(1);
    }
}