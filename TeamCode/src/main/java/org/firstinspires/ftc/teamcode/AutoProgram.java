package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeTest;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

import java.util.List;

@Config
@Autonomous(name = "LimeLightAuto")
public class AutoProgram extends LinearOpMode {

    //Mechanisms
    private Limelight3A limelight3A;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(8.25, 8.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        IntakeTest intake = new IntakeTest();
        Outtake outtake = new Outtake();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Limelight Initialized");
        telemetry.update();
        limelight3A.pipelineSwitch(0);
        waitForStart();
        runtime.reset();
        limelight3A.start();

        Action Line1;
        Action Line2;
        Action Line3;

        Line1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(36,-24),Math.toRadians(0))
                .build();
        Line2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(34,4),Math.toRadians(0))
                .build();
        Line3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(32,30),Math.toRadians(-10))
                .build();
        while (opModeIsActive()) {
            LLResult llresult = limelight3A.getLatestResult();

            if (llresult != null && llresult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llresult.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();

                        if (id == 21) {
                            Actions.runBlocking(
                                    new SequentialAction(
                                            Line3));
                            telemetry.addData("Detected AprilTag ID", id);
                            sleep(30000);

                        } else if (id == 22) {
                            Actions.runBlocking(
                                    new SequentialAction(
                                            Line2));
                            telemetry.addData("Detected Second AprilTag ID", id);
                            sleep(30000);

                        }else if (id == 23){
                            Actions.runBlocking(
                                    new SequentialAction(
                                            Line1
                                    )
                            );
                            telemetry.addData("Detected Third AprilTag ID", id);
                            sleep(30000);
                        }

                    }


                }else {
                    telemetry.addData("Status", "No AprilTags detected");


                }
            }else {
                telemetry.addData("Runtime", runtime.toString());
                telemetry.update();


            }

        }
    }
}