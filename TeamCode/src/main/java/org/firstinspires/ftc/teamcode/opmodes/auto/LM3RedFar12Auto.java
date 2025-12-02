package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class LM3RedFar12Auto {
    public static class Paths {
        //label path field variables and also replace redundant build path coords with a preset variable like shooting pos
        

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.500, 8.450), new Pose(83.500, 17.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70.12))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(100.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.000, 35.000), new Pose(133.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.000, 35.000), new Pose(83.500, 17.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(83.500, 17.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(70.12))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(100.000, 59.400))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.000, 59.400), new Pose(133.000, 59.400))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.000, 59.400), new Pose(129.054, 53.852))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.054, 53.852), new Pose(83.500, 17.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(83.500, 17.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(70.12))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(100.000, 83.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.000, 83.500), new Pose(128.500, 83.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.500, 83.500), new Pose(126.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path14 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.000, 83.500), new Pose(83.500, 17.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.500, 17.000), new Pose(83.500, 17.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(70.12))
                    .build();
        }
    }
}
