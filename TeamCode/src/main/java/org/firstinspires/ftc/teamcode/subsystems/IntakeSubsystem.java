package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        INTAKESTILL_ROLLERSIN, INTAKEOUT_ROLLERSOUT, INTAKEIN_ROLLERSIN, INTAKEOUT_ROLLERSIN, INTAKESTILL_ROLLERSSTILL;
    }
    public IntakeState intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
    private DcMotor intakeWheels;
    private CRServo sideWheel;
    private CRServo sideWheel2;
    public IntakeSubsystem(final HardwareMap hMap) {
        intakeWheels = hMap.get(DcMotor.class, "intake");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sideWheel = hMap.get(CRServo.class, "intakeSide1");
        sideWheel2 = hMap.get(CRServo.class, "intakeSide2");
    }

    public void set(IntakeState state) {
        switch(state) {
            case INTAKESTILL_ROLLERSIN:
                intakeWheels.setPower(0.0);
                sideWheel.setPower(-1.0);
                sideWheel2.setPower(1.0);
                break;
            case INTAKEOUT_ROLLERSOUT:
                intakeWheels.setPower(1.0);
                sideWheel.setPower(-1.0);
                sideWheel2.setPower(1.0);
                break;
            case INTAKEIN_ROLLERSIN:
                intakeWheels.setPower(-1.0);
                sideWheel.setPower(-1.0);
                sideWheel2.setPower(1.0);
                break;
            case INTAKEOUT_ROLLERSIN:
                intakeWheels.setPower(-1.0);
                sideWheel.setPower(1.0);
                sideWheel2.setPower(-1.0);
                break;
            case INTAKESTILL_ROLLERSSTILL:
                intakeWheels.setPower(0.0);
                sideWheel.setPower(0.0);
                sideWheel2.setPower(0.0);
                break;
        }
        intakeState = state;
    }
}