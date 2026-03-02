package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

//import org.firstinspires.ftc.teamcode.subsystems.Light;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RT2GS18", group = "Red Alliance")
public class RT2GS18 extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.7515527950311;
    private static final double START_Y = 134.70807453416143;
    private static final double START_HEADING = 0;

    public RT2GS18() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
                //Light.INSTANCE,
                //Limelight.INSTANCE
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;
        Transfer.INSTANCE.closeGate();

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Order:", "Second Row, Gate x3, First Row");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("End Count:", "6");
        telemetry.addData("Total Count:", "18");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.mode = Turret.Mode.manual;
        Shooter.INSTANCE.mode = Shooter.Mode.odometry;
        //Limelight.INSTANCE.autoUpdateEnabled = false;

        // Enable shooter motors
        Shooter.INSTANCE.flywheelMotor1.getMotor().setMotorEnable();
        Shooter.INSTANCE.flywheelMotor2.getMotor().setMotorEnable();

        new SequentialGroup(
                // Launch Preload
                new FollowPath(paths.LaunchPreload),
                Transfer.INSTANCE.intake(),
                new Delay(1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.R2Launch),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate 1
                new FollowPath(paths.GateOpenCollect1),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate 2
                new FollowPath(paths.GateOpenCollect2),
                new Delay(2),
                new FollowPath(paths.GateLaunch2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

//                // Gate 3
//                new FollowPath(paths.GateOpenCollect3),
//                new Delay(2),
//                new FollowPath(paths.GateLaunch3),
//                Transfer.INSTANCE.openGate(),
//                new Delay(Configuration.SHOOTER_TIME),
//                Transfer.INSTANCE.closeGate(),

                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
//
//        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
//        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

//        // Auto interpolation - continuously update shooter RPM and hood based on distance
//        Shooter.ShotParameters params = Shooter.INSTANCE.getShotParameters(Shooter.INSTANCE.GOAL_DISTANCE);
//        if (params != null) {
//            double interpolatedRPM = params.rpm;
//            double interpolatedHood = params.hoodAngle;
//            Shooter.INSTANCE.setTargetRPM(interpolatedRPM, 100.0);
//            Shooter.INSTANCE.hoodServo.setPosition(interpolatedHood);
//            Shooter.INSTANCE.HOOD_POSITION = interpolatedHood;
//
//            telemetry.addLine("=== Shooter Interpolation ===");
//            telemetry.addData("Goal Distance", Shooter.INSTANCE.GOAL_DISTANCE);
//            telemetry.addData("Interpolated RPM", interpolatedRPM);
//            telemetry.addData("Interpolated Hood", interpolatedHood);
//        }

        //double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
        //double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
        //double TOF = Shooter.INSTANCE.getTOF(goalDistance, Math.toRadians(hoodAngle));

        //Configuration.setAimPointOffset(-X_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF,
        //-Y_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF);

//        double headingDeg = Math.toDegrees(PedroComponent.follower().getPose().getHeading());
//
//        telemetry.addLine("=== Position ===");
//        telemetry.addData("X", PedroComponent.follower().getPose().getX());
//        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
//        telemetry.addData("Heading", headingDeg);
//
//        telemetry.addLine("=== Gate Servos ===");
//        telemetry.addData("Gate 1 Position", Transfer.INSTANCE.servoGate1.getServo().getPosition());
//        telemetry.addData("Gate 2 Position", Transfer.INSTANCE.servoGate2.getServo().getPosition());

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        Turret.INSTANCE.emergencyStop().schedule();
        Transfer.INSTANCE.emergencyStopAll().schedule();
        Shooter.INSTANCE.emergencyStop().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain LaunchPreload;
        public PathChain R2Pickup;
        public PathChain R2Launch;
        public PathChain GateOpenCollect1;
        public PathChain GateLaunch1;
        public PathChain GateOpenCollect2;
        public PathChain GateLaunch2;
        public PathChain GateOpenCollect3;
        public PathChain GateLaunch3;
        public PathChain R1Pickup;
        public PathChain R1Launch;

        public Paths(Follower follower) {
            LaunchPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.752, 134.708),
                                    new Pose(88.11764705882348, 83.84033613445378)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.11764705882348, 83.84033613445378),
                                    new Pose(95.750, 53.703),
                                    new Pose(89.607, 59.487),
                                    new Pose(102.893, 60.244),
                                    new Pose(129.546, 59.487)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.546, 59.487),
                                    new Pose(101.714, 61.227),
                                    new Pose(87.832, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                    .build();

            GateOpenCollect1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.832, 83.992),
                                    new Pose(92.488, 57.326),
                                    new Pose(134.400, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(32))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.400, 55.000),
                                    new Pose(101.714, 61.227),
                                    new Pose(87.832, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(42))
                    .build();

            GateOpenCollect2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.832, 83.992),
                                    new Pose(92.488, 57.326),
                                    new Pose(134.400, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(32))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.400, 55.000),
                                    new Pose(101.714, 61.227),
                                    new Pose(87.832, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(42))
                    .build();

            GateOpenCollect3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.832, 83.992),
                                    new Pose(92.488, 57.326),
                                    new Pose(134.400, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(32))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.400, 55.000),
                                    new Pose(101.714, 61.227),
                                    new Pose(88.588, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(0))
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.588, 83.992),
                                    new Pose(126.672, 83.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.672, 83.723),
                                    new Pose(87.95798319327731, 83.84033613445378)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                    .build();
        }
    }
}