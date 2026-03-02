package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import static dev.nextftc.bindings.Bindings.*;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Shooter", group = "Debug")
public class ShooterDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    // Hood position control (servo position 0-1)
    private double hoodPosition = 0.5;
    private static final double HOOD_INCREMENT = 0.01;

    // RPM control
    private double targetRPM = 0.0;
    private static final double RPM_INCREMENT = 100.0;

    // Shooter mode: MANUAL, AUTO_RPM (manual hood, auto rpm), FULL_AUTO (auto hood and rpm)
    private enum ShooterMode {
        MANUAL,      // Manual hood and RPM
        AUTO_RPM,    // Manual hood, auto RPM from math model
        FULL_AUTO    // Auto hood from getHoodAngle(), auto RPM from math model
    }
    private ShooterMode shooterMode = ShooterMode.MANUAL;

    // Gate state
    private boolean gateOpen = false;

    // Logged values
    private String loggedValues = "No values logged yet";

    public ShooterDebug() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        // Set starting pose to (72, 72, 270 degrees)
        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(270)));
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Set shooter to manual mode on init
        Shooter.INSTANCE.mode = Shooter.Mode.manual;

        // Set turret to odometry mode so it tracks the goal
        Turret.INSTANCE.mode = Turret.Mode.odometry;
        Turret.INSTANCE.turretServo1.getServo().getController().pwmEnable();
        Turret.INSTANCE.turretServo2.getServo().getController().pwmEnable();

        // Enable hood servos on init
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();
        Shooter.INSTANCE.setHoodAngle(56.5); // Middle of range (43-70)
        hoodPosition = Shooter.INSTANCE.HOOD_POSITION;
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();

        // Enable hood servos and set initial position
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();
        Shooter.INSTANCE.setHoodAngle(56.5); // Middle of range (43-70)
        hoodPosition = Shooter.INSTANCE.HOOD_POSITION;

        // Enable BOTH shooter motors explicitly
        Shooter.INSTANCE.flywheelMotor1.getMotor().setMotorEnable();
        Shooter.INSTANCE.flywheelMotor2.getMotor().setMotorEnable();

        // Set shooter to manual mode
        Shooter.INSTANCE.mode = Shooter.Mode.manual;

        // X Button - Cycle through modes: MANUAL -> AUTO_RPM -> FULL_AUTO -> MANUAL
        button(() -> gamepad1.x)
                .whenBecomesTrue(() -> {
                    switch (shooterMode) {
                        case MANUAL:
                            shooterMode = ShooterMode.AUTO_RPM;
                            Shooter.INSTANCE.mode = Shooter.Mode.odometry;
                            break;
                        case AUTO_RPM:
                            shooterMode = ShooterMode.FULL_AUTO;
                            Shooter.INSTANCE.mode = Shooter.Mode.odometry;
                            break;
                        case FULL_AUTO:
                            shooterMode = ShooterMode.MANUAL;
                            Shooter.INSTANCE.mode = Shooter.Mode.manual;
                            break;
                    }
                });

        // Left Bumper - Close gate
        button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.closeGate().schedule();
                    gateOpen = false;
                });

        // Right Bumper - Open gate
        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.openGate().schedule();
                    gateOpen = true;
                });

        // D-Pad Up - Increase hood angle (works in MANUAL and AUTO_RPM modes)
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    if (shooterMode != ShooterMode.FULL_AUTO) {
                        double newAngle = Shooter.INSTANCE.HOOD_ANGLE + 1.0;
                        if (newAngle > 70) newAngle = 70;
                        Shooter.INSTANCE.setHoodAngle(newAngle);
                        hoodPosition = Shooter.INSTANCE.HOOD_POSITION;
                    }
                });

        // D-Pad Down - Decrease hood angle (works in MANUAL and AUTO_RPM modes)
        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    if (shooterMode != ShooterMode.FULL_AUTO) {
                        double newAngle = Shooter.INSTANCE.HOOD_ANGLE - 1.0;
                        if (newAngle < 43) newAngle = 43;
                        Shooter.INSTANCE.setHoodAngle(newAngle);
                        hoodPosition = Shooter.INSTANCE.HOOD_POSITION;
                    }
                });

        // D-Pad Right - Increase RPM (only in MANUAL mode)
//        button(() -> gamepad1.dpad_right)
//                .whenBecomesTrue(() -> {
//                    if (shooterMode == ShooterMode.MANUAL) {
//                        targetRPM += RPM_INCREMENT;
//                        Shooter.INSTANCE.setTargetRPM(targetRPM, 100.0);
//                    }
//                });
//
//        // D-Pad Left - Decrease RPM (only in MANUAL mode)
//        button(() -> gamepad1.dpad_left)
//                .whenBecomesTrue(() -> {
//                    if (shooterMode == ShooterMode.MANUAL) {
//                        targetRPM = Math.max(0, targetRPM - RPM_INCREMENT);
//                        Shooter.INSTANCE.setTargetRPM(targetRPM, 100.0);
//                    }
//                });

        // A Button - Run intake
        button(() -> gamepad1.a)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.stop().schedule());

//        // Y Button - Log values to telemetry (for lookup table)
//        button(() -> gamepad1.y)
//                .whenBecomesTrue(() -> {
//                    double m1VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor1.getMotor().getVelocity());
//                    double m2VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor2.getMotor().getVelocity());
//                    double m1ActualRPM = Shooter.velocityToRPM(m1VelocityTicks);
//                    double m2ActualRPM = Shooter.velocityToRPM(m2VelocityTicks);
//
//                    // Format for easy copy-paste into lookup table
//                    loggedValues = String.format(
//                            "shooterLookupTable.put(%.2f, new ShotParameters(%.2f, %.0f, %.3f));",
//                            Shooter.INSTANCE.GOAL_DISTANCE,
//                            Shooter.INSTANCE.GOAL_DISTANCE,
//                            autoMode ? interpolatedRPM : targetRPM,
//                            Shooter.INSTANCE.HOOD_POSITION
//                    );
//                });
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Update current pose from follower for odometry tracking
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // In FULL_AUTO mode, calculate hood angle from distance using getHoodAngle()
        if (shooterMode == ShooterMode.FULL_AUTO) {
            double distMeters = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;
            double autoHoodAngle = Shooter.INSTANCE.getHoodAngle(distMeters);
            Shooter.INSTANCE.setHoodAngle(autoHoodAngle);
            hoodPosition = Shooter.INSTANCE.HOOD_POSITION;
        }

//        // Keep turret at center position (0.5 = angle 0)
//        Turret.INSTANCE.turretServo.setPosition(0.5);

//        // Auto interpolation mode - use lookup table and spin flywheel
//        if (autoMode) {
//            Shooter.ShotParameters params = Shooter.INSTANCE.getShotParameters(Shooter.INSTANCE.GOAL_DISTANCE);
//            if (params != null) {
//                interpolatedRPM = params.rpm;
//                interpolatedHood = params.hoodAngle;
//                targetRPM = interpolatedRPM;
//                hoodPosition = interpolatedHood;
//                Shooter.INSTANCE.setTargetRPM(interpolatedRPM, 100.0);
//                Shooter.INSTANCE.hoodServo.setPosition(hoodPosition);
//                Shooter.INSTANCE.HOOD_POSITION = hoodPosition;
//            }
//        }

        // Loop timing
        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000 / LOOP_TIME));

        // Mode
        telemetry.addLine();
        telemetry.addData("=== Mode ===", "");
        telemetry.addData("Mode", shooterMode.toString());
        telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");

        // Distance to goal
        telemetry.addLine();
        telemetry.addData("=== Goal Info ===", "");
        telemetry.addData("Robot Position", PedroComponent.follower().getPose());
        telemetry.addData("Config Position", Configuration.CURRENT_POSE);
        telemetry.addData("Distance to Goal (in)", Shooter.INSTANCE.GOAL_DISTANCE);
        telemetry.addData("Distance to Goal (m)", Shooter.INSTANCE.GOAL_DISTANCE * 0.0254);

        // Turret info
        telemetry.addLine();
        telemetry.addData("=== Turret (Odo Mode) ===", "");
        telemetry.addData("Turret Mode", Turret.INSTANCE.mode);
        telemetry.addData("Turret Angle", Turret.INSTANCE.TURRET_ANGLE);
        telemetry.addData("Turret Position", Turret.INSTANCE.TURRET_POSITION);

        // Shooter info
        telemetry.addLine();
        telemetry.addData("=== Shooter ===", "");
        telemetry.addData("Target RPM (manual)", targetRPM);
        telemetry.addData("Calculated RPM (auto)", Shooter.INSTANCE.FLYWHEEL_RPM_GOAL);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Hood Angle (deg)", Shooter.INSTANCE.HOOD_ANGLE);
        telemetry.addData("Auto Hood Angle", Shooter.INSTANCE.getHoodAngle(Shooter.INSTANCE.GOAL_DISTANCE * 0.0254));
        telemetry.addData("Hood Radians", Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE));

        // Actual RPM
        double m1VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor1.getMotor().getVelocity());
        double m2VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor2.getMotor().getVelocity());
//        telemetry.addData("M1 Actual RPM", Shooter.velocityToRPM(m1VelocityTicks));
//        telemetry.addData("M2 Actual RPM", Shooter.velocityToRPM(m2VelocityTicks));

        // Logged values
        telemetry.addLine();
        telemetry.addData("=== Logged (Y) ===", "");
        telemetry.addData("Log", loggedValues);

        // Controls help
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("X Button", "Cycle: MANUAL->AUTO_RPM->FULL_AUTO");
        telemetry.addData("LB/RB", "Close/Open Gate");
        telemetry.addData("D-Pad Up/Down", "Hood Angle +/- (MANUAL/AUTO_RPM)");
        telemetry.addData("D-Pad Right/Left", "RPM +/-100 (MANUAL only)");
        telemetry.addData("A Button", "Run Intake");
        telemetry.addData("Y Button", "Log Current Values");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
