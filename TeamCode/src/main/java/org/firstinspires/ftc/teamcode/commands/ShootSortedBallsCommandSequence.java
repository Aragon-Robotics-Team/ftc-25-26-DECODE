package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootSortedBallsCommandSequence extends SequentialCommandGroup {

    public ShootSortedBallsCommandSequence(ShooterSubsystem shooterSubsystem,
                                           SpindexerSubsystem spindexerSubsystem,
                                           GateSubsystem gateSubsystem, IntakeSubsystem intakeSubsystem,
                                           RobotConstants.BallColors[] targetBallSequence) {

        // 1. GET LIVE DATA (Must be used inside DeferredCommand to work!)
        RobotConstants.BallColors[] current = spindexerSubsystem.getBalls();

        int bestOffset = 0;
        int bestScore = 0;

        // 2. Calculate optimal rotation
        for (int offset = 0; offset < 3; offset++) {
            int score = calculateMatchScore(current, targetBallSequence, offset);
            if (score > bestScore) {
                bestScore = score;
                bestOffset = offset;
            }
        }

        // Optimization: If we have 2 matching, the 3rd is the last one anyway.
        if (bestScore == 2) bestScore = 3;

        // --- PART 1: ALIGNMENT ---
        if (bestOffset > 0) {
            addCommands(
                    new InstantCommand(gateSubsystem::up),
                    new WaitCommand(200),
                    // Move spindexer to alignment (Does not shoot, just rotates)
                    new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestOffset, false, false),
                    new InstantCommand(gateSubsystem::down),
                    new WaitCommand(300)
            );
        } else {
            addCommands(new InstantCommand(gateSubsystem::down));
        }

        // --- PART 2: SHOOT THE STREAK ---
        if (bestScore > 0) {
            final int scoreFinal = bestScore;
            // INSTANTLY update the "Brain" (Array) so the next steps know these balls are gone.
            addCommands(new InstantCommand(() -> {
                for(int k=0; k<scoreFinal; k++) {
                    spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
                    spindexerSubsystem.shiftBallsArrayBy(1);
                }
            }));
            // Physically shoot the balls
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestScore, false, true).withTimeout(3000));
        }

        // --- PART 3: HANDLE REMAINING BALLS (Smart Fallback) ---
        for (int i = bestScore; i < 3; i++) {
            RobotConstants.BallColors target = targetBallSequence[i];

            addCommands(new ConditionalCommand(
                    // TRUE: Ball matches OR We can't find the right ball.
                    // ACTION: Just Shoot (Dump).
                    new InstantCommand(gateSubsystem::down),

                    // FALSE: Ball is wrong AND we have the right ball elsewhere.
                    // ACTION: Swap.
                    new SequentialCommandGroup(
                            new InstantCommand(gateSubsystem::up),
                            new WaitCommand(200),
                            new LoadBallCommand(spindexerSubsystem, target).withTimeout(1500),
                            new InstantCommand(gateSubsystem::down),
                            new WaitCommand(300)
                    ),

                    // THE SMART CONDITION
                    () -> {
                        RobotConstants.BallColors[] balls = spindexerSubsystem.getBalls();
                        boolean isMatch = (balls[2] == target) || (target == RobotConstants.BallColors.UNKNOWN);

                        // Check if the target ball even exists in the spindexer
                        boolean targetExists = false;
                        for (RobotConstants.BallColors b : balls) {
                            if (b == target) {
                                targetExists = true;
                                break;
                            }
                        }

                        // If it matches, OR if the ball is missing entirely... True (Just Shoot).
                        return isMatch || !targetExists;
                    }
            ));

            // Shoot the single ball
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, 1, false, true));
        }

        // Cleanup
        addCommands(new InstantCommand(gateSubsystem::down));
    }

    private int calculateMatchScore(RobotConstants.BallColors[] balls, RobotConstants.BallColors[] targets, int startOffset) {
        int score = 0;
        int[] sequenceIndices = {2, 1, 0}; // Exit is 2, then 1, then 0

        for (int i = 0; i < 3; i++) {
            int currentIndexPointer = (startOffset + i) % 3;
            int actualSlotIndex = sequenceIndices[currentIndexPointer];

            RobotConstants.BallColors ball = balls[actualSlotIndex];
            RobotConstants.BallColors target = targets[i];

            if (ball == target || target == RobotConstants.BallColors.UNKNOWN) {
                score++;
            } else {
                break;
            }
        }
        return score;
    }
}