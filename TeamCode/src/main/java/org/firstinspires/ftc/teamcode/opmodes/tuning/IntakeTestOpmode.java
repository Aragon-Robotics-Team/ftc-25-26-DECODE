package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "Intake Test", group = "Tuning")
public class IntakeTestOpmode extends OpMode {

    private IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSOUT);
        } else if (gamepad1.b) {
            intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN);
        } else {
            intake.set(IntakeSubsystem.IntakeState.STILL);
        }
    }

}