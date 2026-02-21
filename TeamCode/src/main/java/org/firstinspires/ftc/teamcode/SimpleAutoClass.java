package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

public class SimpleAutoClass extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Outtake out = new Outtake();

        waitForStart();

        resetRuntime();
        while (opModeIsActive()) {
            if (getRuntime() < 1) {
                drive.Drive(0.5, 0, 0);
            }
            else if (getRuntime() < 2) {
                drive.Drive(0, 0, 0);
                 out.setMotorSpeed(1);
            }

        }
    }
}
