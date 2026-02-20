package org.firstinspires.ftc;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeTest;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.TransferServo;

@TeleOp
public class RobotCodeouttakechanges extends LinearOpMode {

    TransferServo Servo;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    double forward, strafe, rotate;
    IntakeTest bench;
    boolean motorspeed = false;
    Outtake Bench1;



    @Override
    public void runOpMode() {
         double OUTTAKE_STOP   =  0.00;
         double OUTTAKE_SLOW   = -0.50;
         double OUTTAKE_FAST   = -0.58;
        Servo = new TransferServo(hardwareMap);
        Servo.init(hardwareMap);
        bench = new IntakeTest();
        bench.init(hardwareMap);
        Bench1 = new Outtake();
        Bench1.init(hardwareMap);
        double drivespeed = 1;
        float forwardpos;
        float horizontalpos;
        float headingpos;
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackRight = hardwareMap.get(DcMotor.class, "BackRightWheel");
        BackLeft =hardwareMap.get(DcMotor.class, "BackLeftWheel");

        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            Bench1.setMotorSpeed(OUTTAKE_STOP);

            while (opModeIsActive()) {

                if (gamepad1.x) {
                    motorspeed = false;
                }
                if (gamepad1.y) {
                    motorspeed = true;
                }
                if (!motorspeed) bench.setMotorSpeed(0);
                if (motorspeed)
                    bench.setMotorSpeed(1);

                if (gamepad1.b) {
                    Bench1.setMotorSpeed(OUTTAKE_SLOW);
                } else if (gamepad1.dpad_up) {
                    Bench1.setMotorSpeed(OUTTAKE_FAST);
                } else if (gamepad1.a) {
                    Bench1.setMotorSpeed(OUTTAKE_STOP);

                }

                if(gamepad1.right_bumper){
                    Servo.setServopos(0.9);
                }
                else {
                    Servo.setServopos(0.60);
                }


                telemetry.addData("outtakeMotor", Bench1.getMotorRevs());
                telemetry.addData("Motor Revs", bench.getMotorRevs());
                forwardpos = gamepad1.left_stick_y;
                horizontalpos = -gamepad1.left_stick_x;
                headingpos = -gamepad1.right_stick_x;


                FrontRight.setPower((-headingpos + (forwardpos - horizontalpos)) * drivespeed);
                BackRight.setPower((-headingpos + forwardpos + horizontalpos) * drivespeed);
                FrontLeft.setPower((headingpos + forwardpos + horizontalpos) * drivespeed);
                BackLeft.setPower((headingpos + (forwardpos - horizontalpos)) * drivespeed);


            }

        }

    }




}
