package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {

    private DcMotor motor;
    private double ticksPerRev;

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "outtakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();

    }
    public void setMotorSpeed(double speed) {
        motor.setPower(speed); // Goes from -1.0 to 1.0

    }

    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev;

    }



}
