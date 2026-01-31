package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeTest {
    private DcMotor motor;
    private double ticksPerRev;

    public IntakeTest(){

    }


    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "Intakemotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed); // Goes from -1.0 to 1.0

    }

    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev;

    }

}

