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

@Autonomous(name = "BT2GS18", group = "Blue Alliance")
public class BT2GS18 extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 32.248;
    private static final double START_Y = 134.708;
    private static final double START_HEADING = 180;

    public BT2GS18() {
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

        telemetry.addData("Alliance:", "Blue");
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
                new FollowPath(paths.GateOpenCollect1),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch1),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Gate 2
                new FollowPath(paths.GateOpenCollect2),
                new Delay(2),
                new FollowPath(paths.GateLaunch2),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // Gate 3
                new FollowPath(paths.GateOpenCollect3),
                new Delay(2),
                new FollowPath(paths.GateLaunch3),
                //TODO: Open transfer gate.
                new Delay(Configuration.SHOOTER_TIME),

                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
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
            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(32.248, 134.708),
                                    new Pose(48.250, 53.703),
                                    new Pose(54.393, 59.487),
                                    new Pose(41.107, 60.244),
                                    new Pose(14.454, 59.487)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.454, 59.487),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GateOpenCollect1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(9.600, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.600, 55.000),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))
                    .build();

            GateOpenCollect2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(8.5, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.5, 55.000),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))
                    .build();

            GateOpenCollect3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(8.5, 55.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.5, 55.000),
                                    new Pose(42.286, 61.227),
                                    new Pose(55.412, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.412, 83.992),
                                    new Pose(17.328, 83.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.328, 83.723),
                                    new Pose(48.176, 120.143)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}