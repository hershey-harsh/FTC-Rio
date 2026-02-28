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

@Autonomous(name = "RT1GA", group = "Red Alliance")
public class RT1GA extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.7515527950311;
    private static final double START_Y = 134.70807453416143;
    private static final double START_HEADING = 0;

    public RT1GA() {
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
        telemetry.addData("Order:", "First Row, Gate Open, Second Row, Third Row, Move to Gate");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "False");
        telemetry.addData("End Count:", "6");
        telemetry.addData("Total Count:", "12");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();
        //Turret.INSTANCE.mode = Turret.Mode.odometry;
        //Shooter.INSTANCE.mode = Shooter.Mode.odometry;
        //Limelight.INSTANCE.autoUpdateEnabled = false;

        new SequentialGroup(
                // Preload Launch
                new FollowPath(paths.PreloadLaunch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.PreGateOpen),
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

                // Move to Gate (End)
                new FollowPath(paths.End)
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

//        double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
//        double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
//        double TOF = Shooter.INSTANCE.getTOF(goalDistance, Math.toRadians(hoodAngle));
//
//        Configuration.setAimPointOffset(-X_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF,
//                -Y_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF);

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
        Transfer.INSTANCE.emergencyStop().schedule();
        //Shooter.INSTANCE.emergencyStop().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain PreloadLaunch;
        public PathChain R1Pickup;
        public PathChain R1Launch;
        public PathChain R2Pickup;
        public PathChain PreGateOpen;
        public PathChain GateOpen;
        public PathChain R2Launch;
        public PathChain R3Pickup;
        public PathChain R3Launch;
        public PathChain End;

        public Paths(Follower follower) {
            PreloadLaunch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(111.752, 134.708),
                                    new Pose(93.214, 106.070),
                                    new Pose(100.571, 83.851)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.571, 83.851),
                                    new Pose(128.801, 83.553)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.801, 83.553),
                                    new Pose(98.646, 94.820)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(98.646, 94.820),
                                    new Pose(97.090, 52.416),
                                    new Pose(99.283, 60.764),
                                    new Pose(97.748, 59.354),
                                    new Pose(127.049, 59.627)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PreGateOpen = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.049, 59.627),
                                    new Pose(123.789, 59.534)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GateOpen = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(123.789, 59.534),
                                    new Pose(115.090, 73.127),
                                    new Pose(126.605, 67.576),
                                    new Pose(131.144, 69.058)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.144, 69.058),
                                    new Pose(98.646, 94.820)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            R3Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(98.646, 94.820),
                                    new Pose(97.994, 29.516),
                                    new Pose(100.649, 37.320),
                                    new Pose(99.289, 34.680),
                                    new Pose(128.801, 35.621)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.801, 35.621),
                                    new Pose(98.792, 95.112)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            End = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.792, 95.112),
                                    new Pose(118.490, 70.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();
        }
    }
}