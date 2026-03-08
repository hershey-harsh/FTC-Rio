package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "RT1GS18", group = "Red Alliance")
public class RT1GS18 extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.418;
    private static final double START_Y = 135.709;
    private static final double START_HEADING = 0;

    public RT1GS18() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
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
        telemetry.addData("Order:", "Preload, R1, Launch, R2, Launch, Gate, Launch, Gate, Launch, R3, Launch, Move");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "18");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();
        Turret.INSTANCE.start().schedule();

        new SequentialGroup(
                // First Row (R1)
//                new ParallelGroup(
//                        new SequentialGroup(
//                                new FollowPath(paths.Path12),
//                                new FollowPath(paths.R1Pickup)
//                        ),
//                        new SequentialGroup(
//                                new Delay(2),
//                                Transfer.INSTANCE.openGate(),
//                                new Delay(1),
//                                Transfer.INSTANCE.closeGate()
//                        )
//                ),
                new FollowPath(paths.Path12),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.R2Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 1
                new FollowPath(paths.Gate1Pickup),
                new Delay(2),
                new FollowPath(paths.Gate1Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 2
                new FollowPath(paths.Gate2Pickup),
                new Delay(2),
                new FollowPath(paths.Gate2Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Third Row (R3)
                new FollowPath(paths.R3Pickup),
                new FollowPath(paths.R3Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Move (End)
                new FollowPath(paths.Move)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000 / LOOP_TIME));

        PedroComponent.follower().update();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        double headingDeg = Math.toDegrees(PedroComponent.follower().getPose().getHeading());

        if (Shooter.INSTANCE.mode == Shooter.Mode.odometry) {
            double distMeters = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;

            double hoodRad = Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE);
            double odoTarget = Turret.INSTANCE.ODO_TARGET;
//
            Shooter.INSTANCE.updateKinematics(distMeters, hoodRad);

            double weight = Shooter.INSTANCE.getWeight();
            Configuration.setAimPointOffset(-X_VELOCITY * weight, -Y_VELOCITY * weight);

            double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - odoTarget))
                    + ((X_VELOCITY * 0.0254) * Math.sin(odoTarget));
            double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - odoTarget))
                    + ((X_VELOCITY * 0.0254) * Math.cos(odoTarget));

            double vn = Shooter.INSTANCE.shooterVKinematic() + (vyr * Shooter.vcWeight);
            double vt = Math.sqrt((vn * vn) + (vxr * vxr));

            Shooter.INSTANCE.setHoodAngle(Shooter.INSTANCE.HOOD_ANGLE);
            Configuration.TURRET_OFFSET = 0;
            Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.vMSToRPM(vt) * 2.75; //2.75 weight if stationary
//                Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.getKinematicRPMGoal() * 2.75;
        }

        telemetry.addLine("=== Position ===");
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", headingDeg);

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        Transfer.INSTANCE.emergencyStopAll().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain Path12;
        public PathChain R1Pickup;
        public PathChain R1Launch;
        public PathChain R2Pickup;
        public PathChain R2Launch;
        public PathChain Gate1Pickup;
        public PathChain Gate1Launch;
        public PathChain Gate2Pickup;
        public PathChain Gate2Launch;
        public PathChain R3Pickup;
        public PathChain R3Launch;
        public PathChain Move;

        public Paths(Follower follower) {
            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.418, 135.709),
                                    new Pose(94.127, 83.545)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.127, 83.545),
                                    new Pose(128.500, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.500, 83.600),
                                    new Pose(84.636, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.636, 73.218),
                                    new Pose(100.060, 57.696),
                                    new Pose(95.893, 59.916),
                                    new Pose(94.039, 59.461),
                                    new Pose(131.409, 59.482)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.409, 59.482),
                                    new Pose(84.636, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Gate1Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.636, 73.218),
                                    new Pose(92.488, 57.326),
                                    new Pose(132.673, 61.545)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Gate1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.673, 61.545),
                                    new Pose(84.636, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Gate2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.636, 73.218),
                                    new Pose(92.488, 57.326),
                                    new Pose(132.818, 61.399)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Gate2Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.818, 61.399),
                                    new Pose(84.636, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            R3Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.636, 73.218),
                                    new Pose(101.438, 29.869),
                                    new Pose(90.689, 34.965),
                                    new Pose(97.844, 35.382),
                                    new Pose(131.274, 35.185)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.274, 35.185),
                                    new Pose(84.636, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Move = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.636, 73.218),
                                    new Pose(97.600, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
