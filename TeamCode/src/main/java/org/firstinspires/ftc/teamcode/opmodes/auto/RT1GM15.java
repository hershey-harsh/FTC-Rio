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
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Configuration;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/// IMPORT INTO PEDRO PATHING SIMULATOR TO SEE PATHS

@Autonomous(name = "RT1GM15", group = "Red Alliance")
public class RT1GM15 extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.7515527950311;
    private static final double START_Y = 134.70807453416143;
    private static final double START_HEADING = 0;

    public RT1GM15() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE)
                //Light.INSTANCE,
                //Limelight.INSTANCE,
                //Shooter.INSTANCE,
                //Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Order:", "Launch, First Row, Launch, Second Row, Gate Open, Launch, Third Row, Launch, Gate Open & Collect, Launch");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "Semi");
        telemetry.addData("End Count:", "idk yet");
        telemetry.addData("Total Count:", "15");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();
        //Turret.INSTANCE.mode = Turret.Mode.odometry;
        //Shooter.INSTANCE.mode = Shooter.Mode.odometry;
        //Limelight.INSTANCE.autoUpdateEnabled = false;

        new SequentialGroup(
                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.PreGateOpen1),
                new FollowPath(paths.PreGateOpen2),
                new FollowPath(paths.GateOpen),
                new Delay(1),
                new FollowPath(paths.R2Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Third Row (R3)
                new FollowPath(paths.R3Pickup),
                new FollowPath(paths.R3Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Gate Open & Collect
                new FollowPath(paths.GateOpenCollect),
                new Delay(2),
                new FollowPath(paths.GateLaunch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        //double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
        //double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
        //double TOF = Shooter.INSTANCE.getTOF(goalDistance, Math.toRadians(hoodAngle));

        //Configuration.setAimPointOffset(-X_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF,
        //-Y_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF);

        double headingDeg = Math.toDegrees(PedroComponent.follower().getPose().getHeading());

        telemetry.addLine("=== Position ===");
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", headingDeg);

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        //Turret.INSTANCE.emergencyStop().schedule();
        Transfer.INSTANCE.emergencyStopAll().schedule();
        //Shooter.INSTANCE.emergencyStop().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain R1Pickup;
        public PathChain R1launch;
        public PathChain R2Pickup;
        public PathChain PreGateOpen1;
        public PathChain PreGateOpen2;
        public PathChain GateOpen;
        public PathChain R2Launch;
        public PathChain R3Pickup;
        public PathChain R3Launch;
        public PathChain GateOpenCollect;
        public PathChain GateLaunch;

        public Paths(Follower follower) {
            R1Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(111.752, 134.708),
                                    new Pose(95.674, 78.371),
                                    new Pose(95.246, 82.514),
                                    new Pose(93.372, 84.211),
                                    new Pose(128.500, 83.496)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.500, 83.496),
                                    new Pose(86.500, 83.697)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.500, 83.697),
                                    new Pose(103.613, 53.244),
                                    new Pose(94.645, 61.811),
                                    new Pose(101.485, 59.055),
                                    new Pose(128.500, 59.639)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PreGateOpen1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.500, 59.639),
                                    new Pose(123.697, 59.639)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PreGateOpen2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.697, 59.639),
                                    new Pose(123.697, 63.540)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GateOpen = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.697, 63.540),
                                    new Pose(128.149, 63.540)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.149, 63.540),
                                    new Pose(86.500, 83.697)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.500, 83.697),
                                    new Pose(99.786, 30.391),
                                    new Pose(95.761, 35.172),
                                    new Pose(96.071, 35.508),
                                    new Pose(128.500, 35.538)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.500, 35.538),
                                    new Pose(86.500, 83.697)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GateOpenCollect = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.500, 83.697),
                                    new Pose(92.488, 57.326),
                                    new Pose(134.400, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
                    .build();

            GateLaunch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.400, 55.000),
                                    new Pose(87.950, 111.739)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}