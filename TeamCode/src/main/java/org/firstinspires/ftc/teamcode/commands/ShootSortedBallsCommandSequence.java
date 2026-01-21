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

        RobotConstants.BallColors[] current = spindexerSubsystem.getBalls();
        int bestOffset = 0;
        int bestScore = 0;

        for (int offset = 0; offset < 3; offset++) {
            int score = calculateMatchScore(current, targetBallSequence, offset);
            if (score > bestScore) {
                bestScore = score;
                bestOffset = offset;
            }
        }
        if (bestScore == 2) bestScore = 3;

        // --- PART 2: INITIAL POSITIONING ---
        if (bestOffset > 0) {
            addCommands(
                    new InstantCommand(gateSubsystem::up),
                    new WaitCommand(250), // Replaced WaitForGate with timed wait
                    new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestOffset, false),
                    new InstantCommand(gateSubsystem::down),
                    new WaitCommand(400)
            );
        } else {
            // ONLY command down if we aren't already sure it's down.
            // This prevents the "jitter" if the gate is already down from setup.
            addCommands(new InstantCommand(gateSubsystem::down));
        }

        // Shoot the initial matching streak
        if (bestScore > 0) {
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, bestScore, false).withTimeout(2500));
        }

        // --- PART 3: REMAINING BALLS ---
        for (int i = bestScore; i < 3; i++) {
            RobotConstants.BallColors target = targetBallSequence[i];

            addCommands(new ConditionalCommand(
                    // If ball matches, just ensure gate is down
                    new InstantCommand(gateSubsystem::down),
                    // If ball doesn't match, swap it
                    new SequentialCommandGroup(
                            new InstantCommand(gateSubsystem::up),
                            new WaitCommand(250),
                            new LoadBallCommand(spindexerSubsystem, target).withTimeout(1500),
                            new InstantCommand(gateSubsystem::down),
                            new WaitCommand(400)
                    ),
                    () -> spindexerSubsystem.getBalls()[2] == target || target == RobotConstants.BallColors.UNKNOWN
            ));

            // Execute the single shot
            addCommands(new MoveSpindexerAndUpdateArrayCommand(spindexerSubsystem, gateSubsystem, 1, false));
        }

        // Final safety: Gate stays down until next intake cycle starts
        addCommands(new InstantCommand(gateSubsystem::down));
    }
    // ... calculateMatchScore remains the same ...
    /**
     * Helper: How many balls match the target sequence if we start at 'offset'?
     * Spindexer Order Assumption: Exit is Index 2. Order of arrival is 2 -> 1 -> 0.
     */
    private int calculateMatchScore(RobotConstants.BallColors[] balls,
                                    RobotConstants.BallColors[] targets,
                                    int startOffset) {
        int score = 0;
        // Spindexer indices are usually 0, 1, 2.
        // If we rotate 'startOffset' times, the effective start index shifts.
        // Let's assume standard rotation:
        // Offset 0: Sequence is balls[2], balls[1], balls[0]
        // Offset 1: Sequence is balls[1], balls[0], balls[2]
        // Offset 2: Sequence is balls[0], balls[2], balls[1]

        int[] sequenceIndices = {2, 1, 0};

        for (int i = 0; i < 3; i++) {
            // Calculate which ball slot is at the exit for step 'i'
            // given our initial 'startOffset'
            int currentIndexPointer = (startOffset + i) % 3;
            int actualSlotIndex = sequenceIndices[currentIndexPointer];

            RobotConstants.BallColors ball = balls[actualSlotIndex];
            RobotConstants.BallColors target = targets[i];

            if (ball == target || target == RobotConstants.BallColors.UNKNOWN) {
                score++;
            } else {
                // Optimization Strategy:
                // If we break the streak, we stop counting.
                // We prioritize a perfect START over a high total count.
                // e.g. Match-Match-Fail (Score 2) is better than Fail-Match-Match (Score 0)
                break;
            }
        }
        return score;
    }
}