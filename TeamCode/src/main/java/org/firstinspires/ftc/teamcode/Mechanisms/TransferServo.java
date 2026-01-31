package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


    public class TransferServo {
    private Servo servopos;

    public TransferServo(HardwareMap hwMap)
    {
        init(hwMap);
    }
    public void init(HardwareMap hardwareMap) {
        servopos = hardwareMap.get(Servo.class, "TransferServo");


        servopos.setPosition(0.60);
    }


    public void setServopos (double angle) {
        servopos.setPosition(angle);

    }



}
