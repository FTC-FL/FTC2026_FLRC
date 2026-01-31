package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeTest;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.TransferServo;

@TeleOp
public class RobotCode extends LinearOpMode {

    TransferServo Servo;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    double forward, strafe, rotate;
    IntakeTest bench;
    boolean motorspeed = false;
    Outtake Bench1;

    boolean outtakemotorspeed = false;

    @Override
    public void runOpMode() {
        Servo = new TransferServo(hardwareMap);
        bench = new IntakeTest();
        Bench1 = new Outtake();
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


            while (opModeIsActive()) {

                if (gamepad1.x) {
                    motorspeed = false;
                }
                if (gamepad1.y) {
                    motorspeed = true;
                }
                if (motorspeed == false) bench.setMotorSpeed(0);
                if (motorspeed == true)
                    bench.setMotorSpeed(1);
                if (gamepad1.a) {
                    outtakemotorspeed = false;
                }
                if (gamepad1.b) {
                    outtakemotorspeed = true;
                }
                if (outtakemotorspeed == false) Bench1.setMotorSpeed(0);
                if (outtakemotorspeed == true)
                    Bench1.setMotorSpeed(0.6);

                if(gamepad1.right_bumper){
                    Servo.setServopos(-0.4);
                }
                else {
                    Servo.setServopos(0.4);
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
