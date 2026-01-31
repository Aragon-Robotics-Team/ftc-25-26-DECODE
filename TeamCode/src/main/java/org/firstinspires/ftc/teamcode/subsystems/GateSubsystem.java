package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class GateSubsystem extends SubsystemBase {
    ServoEx gate;
    AnalogInput gateEncoder;
    public double UP = 0.21;
    public double DOWN = UP - 0.15;
//    public final double UP_VOLTAGE = 1.67; //Volts
//    public final double DOWN_VOLTAGE = UP_VOLTAGE - 0.58; //Volts

    public GateSubsystem(final HardwareMap hMap) {
        gate = new ServoEx(hMap, "gate");
        gateEncoder = hMap.get(AnalogInput.class, "gateEncoder");
        gate.setInverted(true);
    }
    public enum GateState {
        UP, DOWN;
    }
    public GateState gateState;
    public void up() {
        gate.set(UP);
        gateState = GateState.UP;
    }
    public void down() {
        gate.set(DOWN);
        gateState = GateState.DOWN;
    }
    public double getUp() {
        return UP;
    }
    public void setAdjustment(double upOffset) {
        UP += upOffset;
    }
    public double getEncoderVoltage() {
        return gateEncoder.getVoltage();
    }
    public boolean isAtTarget() {
        //is gate at position
        //add wait for gate finish command
//        return Math.abs(gateEncoder.getVoltage() - UP_VOLTAGE) < 0.05 || Math.abs(gateEncoder.getVoltage() - DOWN_VOLTAGE) < 0.05;
        return false;
    }

}
