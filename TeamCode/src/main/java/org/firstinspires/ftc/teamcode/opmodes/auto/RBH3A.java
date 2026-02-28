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

@Autonomous(name = "RBH3A", group = "Red Alliance")
public class RBH3A extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 87.61344537815125;
    private static final double START_Y = 8.30252100840336;
    private static final double START_HEADING = 0;

    public RBH3A() {
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
        telemetry.addData("Side:", "Bottom");
        telemetry.addData("Order:", "Launch, Third Row, Launch, Human Player & Launch x3, Move");
        telemetry.addData("Gate:", "False");
        telemetry.addData("Solo:", "False");
        telemetry.addData("End Count:", "Dependent");
        telemetry.addData("Total Count:", "6+");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();
        //Turret.INSTANCE.mode = Turret.Mode.odometry;
        //Shooter.INSTANCE.mode = Shooter.Mode.odometry;
        //Limelight.INSTANCE.autoUpdateEnabled = false;

        new SequentialGroup(// Third Row (R3)
                new Delay(Configuration.SHOOTER_TIME),
                new FollowPath(paths.R3Pickup),
                new FollowPath(paths.R3Launch),
                new Delay(Configuration.SHOOTER_TIME),

                // Park Cycle 1
                new FollowPath(paths.ParkPrePickup1),
                new FollowPath(paths.ParkBackup1),
                new FollowPath(paths.ParkPickup1),
                new FollowPath(paths.ParkLaunch),
                new Delay(Configuration.SHOOTER_TIME),

                // Park Cycle 2
                new FollowPath(paths.ParkPrePickup2),
                new FollowPath(paths.ParkBackup2),
                new FollowPath(paths.ParkPickup2),
                new FollowPath(paths.ParkLaunch),
                new Delay(Configuration.SHOOTER_TIME),

                // Park Cycle 3
                new FollowPath(paths.ParkPrePickup3),
                new FollowPath(paths.ParkBackup3),
                new FollowPath(paths.ParkPickup3),
                new FollowPath(paths.ParkLaunch),
                new Delay(Configuration.SHOOTER_TIME),

                // Move
                new FollowPath(paths.Move)
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
        public PathChain R3Pickup;
        public PathChain R3Launch;
        public PathChain ParkPrePickup1;
        public PathChain ParkBackup1;
        public PathChain ParkPickup1;
        public PathChain ParkLaunch;
        public PathChain ParkPrePickup2;
        public PathChain ParkBackup2;
        public PathChain ParkPickup2;
        public PathChain ParkPrePickup3;
        public PathChain ParkBackup3;
        public PathChain ParkPickup3;
        public PathChain Move;

        public Paths(Follower follower) {
            R3Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.613, 8.303),
                                    new Pose(94.174, 45.256),
                                    new Pose(92.233, 32.886),
                                    new Pose(93.628, 35.609),
                                    new Pose(128.801, 35.621)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.801, 35.621),
                                    new Pose(87.613, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPrePickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkBackup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(115.000, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(115.000, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkLaunch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(87.613, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPrePickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkBackup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(115.000, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(115.000, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPrePickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkBackup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(115.000, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ParkPickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(115.000, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Move = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(105.286, 32.714)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}