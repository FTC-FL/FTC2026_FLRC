package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;

import messages.DriveCommandMessage;
import messages.MecanumCommandMessage;

public class MecanumDrive {
    public Pose2d pose = new Pose2d(0, 0, 0);          // current estimated pose — YOU NEED A LOCALIZER TO UPDATE THIS
    public Object localizer;
    public LazyImu lazyImu;

    public VoltageSensor voltageSensor;
    public DcMotorEx FrontRight;
    public DcMotorEx FrontLeft;
    public DcMotorEx BackRight;
    public DcMotorEx BackLeft;
    private IMU imu;

    public MecanumDrive(HardwareMap hwMap)
    {
        init(hwMap);
    }

    public MecanumDrive(HardwareMap hwMap, Pose2d pose)
    {
        init(hwMap);
    }

    public void setDrivePowers(PoseVelocity2d poseVelocity2d) {
    }

    public void updatePoseEstimate() {

    }

    public DriveLocalizer createDriveLocalizer() {
        return new DriveLocalizer();
    }

    public static class Params {
        // Recommended defaults — tune later
        public double kS = 1.5;               // static friction feedforward
        public double kV = 0.0;               // velocity feedforward (usually in V/(in/s) — will be divided by inPerTick)
        public double kA = 0.0;               // acceleration feedforward
        public double inPerTick = 1.0;        // ← change this during tuning!!

        // PID gains (very rough starting points — tune these!)
        public double axialGain      = 5.0;
        public double lateralGain    = 5.0;
        public double headingGain    = 2.0;
        public double axialVelGain   = 0.5;
        public double lateralVelGain = 0.5;
        public double headingVelGain = 0.2;
        public double maxWheelVel;
        public double minProfileAccel;
        public double maxProfileAccel;
    }

    private double prevTs = -1;
    public static final Params PARAMS = new Params();

    private void init(HardwareMap hwMap) {
        DcMotorEx FrontRight = hwMap.get(DcMotorEx.class, "FrontRightWheel");
        DcMotorEx FrontLeft = hwMap.get(DcMotorEx.class, "FrontLeftWheel");
        DcMotorEx BackRight = hwMap.get(DcMotorEx.class, "BackRightWheel");
        DcMotorEx BackLeft =hwMap.get(DcMotorEx.class, "BackLeftWheel");

        FrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        imu = hwMap.get(IMU.class, "imu");
        voltageSensor = hwMap.voltageSensor.iterator().next();  // or get a specific one
        localizer = createDriveLocalizer();

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(RevOrientation));


    }
    public void Drive(double forward, double strafe, double rotate) {
        double FrontLeftPower = forward + strafe + rotate;
        double BackLeftPower = forward - strafe + rotate;
        double FrontRightPower = forward - strafe - rotate;
        double BackRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(FrontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRightPower));
        maxPower = Math.max(maxPower, Math.abs(BackRightPower));

        FrontLeft.setPower(maxSpeed * (FrontLeftPower / maxPower));
        BackLeft.setPower(maxSpeed * (BackLeftPower / maxPower));
        FrontRight.setPower(maxSpeed * (FrontRightPower / maxPower));
        BackRight.setPower(maxSpeed * (BackRightPower / maxPower));
    }
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public final MecanumKinematics kinematics = new MecanumKinematics(
            GlobalConstants.inPerTick * GlobalConstants.trackWidthTicks, GlobalConstants.inPerTick / GlobalConstants.lateralInPerTick);
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            GlobalConstants.maxAngVel, -GlobalConstants.maxAngAccel, GlobalConstants.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(GlobalConstants.maxWheelVel),

                    new AngularVelConstraint(GlobalConstants.maxAngVel)));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(GlobalConstants.minProfileAccel, GlobalConstants.maxProfileAccel);

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                FrontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    GlobalConstants.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            FrontLeft.setPower(leftFrontPower);
            BackLeft.setPower(leftBackPower);
            BackRight.setPower(rightBackPower);
            FrontRight.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            return true;
        }

        private PoseVelocity2d updatePoseEstimate() {

            return null;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                FrontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            FrontLeft.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            BackLeft.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            BackRight.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            FrontRight.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            return true;
        }

        private PoseVelocity2d updatePoseEstimate() {
            return null;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public static class DriveLocalizer {
        public Encoder FrontLeft;
        public Encoder BackLeft;
        public Encoder FrontRight;
        public Encoder BackRight;

        private DriveLocalizer() {
        }
    }
}