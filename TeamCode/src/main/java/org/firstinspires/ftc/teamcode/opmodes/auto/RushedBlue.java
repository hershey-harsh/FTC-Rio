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

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RushedBlue", group = "Blue Alliance")
public class RushedBlue extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position (mirrored for blue)
    private static final double START_X = 32.248;
    private static final double START_Y = 134.708;
    private static final double START_HEADING = 180;

    public RushedBlue() {
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
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;
        Transfer.INSTANCE.closeGate();

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
        Turret.INSTANCE.mode = Turret.Mode.manual;
        Shooter.INSTANCE.mode = Shooter.Mode.odometry;

        // Enable shooter motors
        Shooter.INSTANCE.flywheelMotor1.getMotor().setMotorEnable();
        Shooter.INSTANCE.flywheelMotor2.getMotor().setMotorEnable();

        new SequentialGroup(
                // Launch Preload
                new FollowPath(paths.LaunchPreload),
                Transfer.INSTANCE.intake(),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.outtake(),
                Transfer.INSTANCE.closeGate(),
                Transfer.INSTANCE.intake(),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.R2Launch),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.outtake(),
                Transfer.INSTANCE.closeGate(),
                Transfer.INSTANCE.intake(),

                // Gate 1
                new FollowPath(paths.GateOpenCollect1),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.outtake(),
                Transfer.INSTANCE.closeGate(),
                Transfer.INSTANCE.intake(),

                // Gate 2
                new FollowPath(paths.GateOpenCollect2),
                new Delay(2),
                new FollowPath(paths.GateLaunch2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.outtake(),
                Transfer.INSTANCE.closeGate(),
                Transfer.INSTANCE.intake(),

                // First Row (R1)
                new FollowPath(paths.R1Pickup),
                new FollowPath(paths.R1Launch),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.outtake(),
                Transfer.INSTANCE.closeGate(),
                Transfer.INSTANCE.intake()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

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

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        Turret.INSTANCE.emergencyStop().schedule();
        Transfer.INSTANCE.emergencyStop().schedule();
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
                                    new Pose(32.248, 134.708),
                                    new Pose(55.882, 83.840)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.882, 83.840),
                                    new Pose(48.250, 53.703),
                                    new Pose(54.393, 59.487),
                                    new Pose(41.107, 60.244),
                                    new Pose(8.613445378151296, 55.487)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.613445378151296, 59.487),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();

            GateOpenCollect1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(6.4, 56.454)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(148))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(7.4, 56.454),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(138))
                    .build();

            GateOpenCollect2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(6.4, 56.454)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(148))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(7.4, 56.454),
                                    new Pose(42.286, 61.227),
                                    new Pose(56.168, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(138))
                    .build();

            GateOpenCollect3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.168, 83.992),
                                    new Pose(51.512, 57.326),
                                    new Pose(6.4, 56.454)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(148))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(7.4, 56.454),
                                    new Pose(42.286, 61.227),
                                    new Pose(55.412, 83.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))
                    .build();

            R1Pickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.412, 83.992),
                                    new Pose(14.454, 83.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.454, 83.723),
                                    new Pose(56.34453781512606, 97.30252100840335)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();
        }
    }
}
