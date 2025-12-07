package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFarAutoPaths {
    //label path field variables and also replace redundant build path coords with a preset variable like shooting pos

    Pose shootingSpot = new Pose(83.5, 17);
    //for reference, SZ is shooting zone and C# is Cycle #

    public PathChain toSZ;
    public PathChain toC1;
    public PathChain intakeC1;
    public PathChain returnToSZC1;
    public PathChain toShootingAngle1;
    public PathChain toC2;
    public PathChain intakeC2;
    public PathChain adjustForWallC2;
    public PathChain returnToSZC2;
    public PathChain toShootingAngle2;
    public PathChain toC3;
    public PathChain intakeC3;
    public PathChain adjustForWallC3;
    public PathChain returnToSZC3;
    public PathChain toShootingAngle3;
    public PathChain moveOffLine;

    public RedFarAutoPaths(Follower follower) {
        toSZ = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.500, 8.450), shootingSpot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70.12))
                .build();

        toC1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, new Pose(100.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        intakeC1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 35.000), new Pose(133.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        returnToSZC1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.000, 35.000), shootingSpot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        toShootingAngle1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, shootingSpot)
                )
                .setConstantHeadingInterpolation(Math.toRadians(70.12))
                .build();

        toC2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, new Pose(100.000, 59.400))
                )
                .setTangentHeadingInterpolation()
                .build();

        intakeC2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 59.400), new Pose(133.000, 59.400))
                )
                .setTangentHeadingInterpolation()
                .build();

        adjustForWallC2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.000, 59.400), new Pose(129.054, 53.852))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnToSZC2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.054, 53.852), shootingSpot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        toShootingAngle2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, shootingSpot)
                )
                .setConstantHeadingInterpolation(Math.toRadians(70.12))
                .build();

        toC3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, new Pose(100.000, 83.500))
                )
                .setTangentHeadingInterpolation()
                .build();

        intakeC3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 83.500), new Pose(128.500, 83.500))
                )
                .setTangentHeadingInterpolation()
                .build();

        adjustForWallC3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128.500, 83.500), new Pose(126.000, 83.500))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnToSZC3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.000, 83.500), shootingSpot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        toShootingAngle3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingSpot, shootingSpot)
                )
                .setConstantHeadingInterpolation(Math.toRadians(70.12))
                .build();
        moveOffLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.500, 17.000), new Pose(84.000, 33.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }
}
