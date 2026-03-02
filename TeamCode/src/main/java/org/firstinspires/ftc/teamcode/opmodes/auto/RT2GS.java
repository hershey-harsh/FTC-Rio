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

@Autonomous(name = "RT2GS", group = "Red Alliance")
public class RT2GS extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.7515527950311;
    private static final double START_Y = 134.70807453416143;
    private static final double START_HEADING = 0;

    public RT2GS() {
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
        telemetry.addData("Order:", "Second Row, Gate x2, First Row, Move to Gate");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("End Count:", "6");
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
                // Second Row (R2)
                //TODO: Open transfer gate.
                //TODO: Parallel shooting while going to second row.
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.R2Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Gate 1
                new FollowPath(paths.Gate1Pickup),
                new Delay(2.75),
                new FollowPath(paths.Gate1Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Gate 2
                new FollowPath(paths.Gate2Pickup),
                new Delay(2.75),
                new FollowPath(paths.Gate2Launch),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
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
        public PathChain R2Pickup;
        public PathChain R2Launch;
        public PathChain Gate1Pickup;
        public PathChain Gate1Launch;
        public PathChain Gate2Pickup;
        public PathChain Gate2Launch;
        public PathChain R1Pickup;
        public PathChain R1Launch;
        public PathChain End;

        public Paths(Follower follower) {
            R2Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(111.752, 134.708),
                            new Pose(99.925, 52.870),
                            new Pose(91.745, 59.135),
                            new Pose(100.323, 59.663),
                            new Pose(134.000, 59.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(134.000, 59.500),
                            new Pose(109.922, 58.315),
                            new Pose(87.807, 83.553)
                    )
            ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Gate1Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(87.807, 83.553),
                            new Pose(109.786, 51.801),
                            new Pose(129.242, 59.180)
                    )
            ).setTangentHeadingInterpolation()
                    .build();

            Gate1Launch = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(129.242, 59.180),
                            new Pose(109.842, 56.416),
                            new Pose(87.807, 83.553)
                    )
            ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Gate2Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(87.807, 83.553),
                            new Pose(109.786, 51.801),
                            new Pose(129.242, 59.180)
                    )
            ).setTangentHeadingInterpolation()
                    .build();

            Gate2Launch = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(129.242, 59.180),
                            new Pose(109.842, 56.416),
                            new Pose(87.807, 83.553)
                    )
            ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87.807, 83.553),
                            new Pose(128.100, 83.400)
                    )
            ).setTangentHeadingInterpolation()
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(128.100, 83.400),
                            new Pose(87.807, 83.553)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            End = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87.807, 83.553),
                            new Pose(108.453, 68.199)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}