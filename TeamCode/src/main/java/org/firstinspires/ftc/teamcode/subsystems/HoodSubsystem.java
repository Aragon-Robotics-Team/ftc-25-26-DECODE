package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private Servo hood;
    public HoodSubsystem(final HardwareMap hMap) {
        hood = hMap.get(Servo.class, "Hood"); //change to whatever rev thing says
    }

    //if you wanted to put the math logic here instead of teleop I think you would do something like this where
    //you just past in the pos or velocity and it adjust the thing
    /*public void setHoodAngleBasedOnVelocity() {
        // i suck at math so i have no idea how to do this without any researching you use look up table or something
    }*/
    
    public void setHoodAngle(double angle) {
        hood.setPosition(angle);
    }
    public double getHoodAngle() {
        return hood.getPosition();
    }


}
