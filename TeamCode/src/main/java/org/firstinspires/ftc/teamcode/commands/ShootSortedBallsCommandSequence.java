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

        // 1. GET LIVE DATA (Works only if wrapped in DeferredCommand)
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
        // Just shoot all 3 to save time.
        if (bestScore == 2) bestScore = 3;

        // --- PART 1: ALIGNMENT ---
        // Rotate spindexer so the first correct ball is at the exit (Index 2)
        if (bestOffset > 0) {
            addCommands(
                    new InstantCommand(gateSubsystem::up),
                    new WaitCommand(200),
                    // Move WITHOUT shooting (Gate is UP). This shifts the array but keeps the balls physically.
                    new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestOffset, false),
                    new InstantCommand(gateSubsystem::down),
                    new WaitCommand(300)
            );
        } else {
            // Ensure gate is down if we are already aligned
            addCommands(new InstantCommand(gateSubsystem::down));
        }

        // --- PART 2: SHOOT THE STREAK ---
        // If we have a streak of matches (e.g., 3 balls), shoot them continuously.
        if (bestScore > 0) {
            // FIX: Manually update the array "brain" to reflect that these balls are about to be gone.
            // This prevents the "Ghost Ball" bug where the robot thinks it still has balls after shooting.
            final int scoreFinal = bestScore;
            addCommands(new InstantCommand(() -> {
                for(int k=0; k<scoreFinal; k++) {
                    // We simulate clearing the exit slot 'k' times
                    spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
                    spindexerSubsystem.shiftBallsArrayBy(1);
                }
            }));

            // Physically shoot the balls
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestScore, false).withTimeout(3000));
        }

        // --- PART 3: HANDLE REMAINING BALLS ---
        // Loop only for the balls we haven't shot yet
        for (int i = bestScore; i < 3; i++) {
            RobotConstants.BallColors target = targetBallSequence[i];

            addCommands(new ConditionalCommand(
                    // TRUE: Ball at exit matches target.
                    // Just Ensure gate is down (almost instant) and shoot.
                    new InstantCommand(gateSubsystem::down),

                    // FALSE: Ball does NOT match. We must swap it.
                    // Gate Up -> Load Correct Ball -> Gate Down
                    new SequentialCommandGroup(
                            new InstantCommand(gateSubsystem::up),
                            new WaitCommand(200),
                            new LoadBallCommand(spindexerSubsystem, target).withTimeout(1500),
                            new InstantCommand(gateSubsystem::down),
                            new WaitCommand(300)
                    ),

                    // The Condition: Check the LIVE array state
                    () -> {
                        RobotConstants.BallColors[] balls = spindexerSubsystem.getBalls();
                        // Check if ball at exit (index 2) matches target OR if we are skipping (UNKNOWN)
                        return balls[2] == target || target == RobotConstants.BallColors.UNKNOWN;
                    }
            ));

            // Shoot the single ball
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, 1, false));
        }

        // Cleanup: Reset gate for next cycle
        addCommands(new InstantCommand(gateSubsystem::down));
    }

    // [Diagram of Spindexer Rotation Logic]
    // Index 2 is EXIT.
    // Offset 0: Sequence [2, 1, 0]
    // Offset 1: Sequence [1, 0, 2] (Right Rotation)
    // Offset 2: Sequence [0, 2, 1]
    private int calculateMatchScore(RobotConstants.BallColors[] balls, RobotConstants.BallColors[] targets, int startOffset) {
        int score = 0;
        int[] sequenceIndices = {2, 1, 0};

        for (int i = 0; i < 3; i++) {
            // Determine which physical slot will be at the exit at step 'i'
            int currentIndexPointer = (startOffset + i) % 3;
            int actualSlotIndex = sequenceIndices[currentIndexPointer];

            RobotConstants.BallColors ball = balls[actualSlotIndex];
            RobotConstants.BallColors target = targets[i];

            if (ball == target || target == RobotConstants.BallColors.UNKNOWN) {
                score++;
            } else {
                break; // Streak broken
            }
        }
        return score;
    }
}