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

@TeleOp(name = "SOTM", group = "Debug")
public class SotmDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    // Velocity tracking for shoot-on-the-move
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;

    // Gate state
    private boolean gateOpen = false;

    // SOTM enabled state
    private boolean sotmEnabled = true;

    public SotmDebug() {
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

        // Set shooter to odometry mode (full auto)
        Shooter.INSTANCE.mode = Shooter.Mode.odometry;

        // Set turret to odometry mode so it tracks the goal
        Turret.INSTANCE.mode = Turret.Mode.odometry;
        Turret.INSTANCE.turretServo1.getServo().getController().pwmEnable();
        Turret.INSTANCE.turretServo2.getServo().getController().pwmEnable();

        // Enable hood servos on init
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();
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

        // Enable hood servos
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();

        // Enable BOTH shooter motors explicitly
        Shooter.INSTANCE.flywheelMotor1.getMotor().setMotorEnable();
        Shooter.INSTANCE.flywheelMotor2.getMotor().setMotorEnable();

        // Set shooter to odometry mode (full auto)
        Shooter.INSTANCE.mode = Shooter.Mode.odometry;

        // Enable turret servos and set to odometry mode
        Turret.INSTANCE.turretServo1.getServo().getController().pwmEnable();
        Turret.INSTANCE.turretServo2.getServo().getController().pwmEnable();
        Turret.INSTANCE.mode = Turret.Mode.odometry;

        // X Button - Toggle SOTM on/off
        button(() -> gamepad1.x)
                .whenBecomesTrue(() -> {
                    sotmEnabled = !sotmEnabled;
                    if (!sotmEnabled) {
                        // Reset aim point offset when SOTM is disabled
                        Configuration.setAimPointOffset(0, 0);
                    }
                });

//        // Left Bumper - Close gate
//        button(() -> gamepad1.left_bumper)
//                .whenBecomesTrue(() -> {
//                    Transfer.INSTANCE.closeGate().schedule();
//                    gateOpen = false;
//                });
//
//        // Right Bumper - Open gate
//        button(() -> gamepad1.right_bumper)
//                .whenBecomesTrue(() -> {
//                    Transfer.INSTANCE.openGate().schedule();
//                    gateOpen = true;
//                });

        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        // A Button - Run intake
        button(() -> gamepad2.a)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.stop().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Update current pose from follower for odometry tracking
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Ensure turret stays in odometry mode and PWM stays enabled
        Turret.INSTANCE.mode = Turret.Mode.odometry;
        Turret.INSTANCE.turretServo1.getServo().getController().pwmEnable();
        Turret.INSTANCE.turretServo2.getServo().getController().pwmEnable();

        // Auto hood angle calculation from distance
        double distMeters = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;
        double autoHoodAngle = Shooter.INSTANCE.getHoodAngle(distMeters);
        Shooter.INSTANCE.setHoodAngle(autoHoodAngle);

        // SOTM - Shoot on the move velocity compensation
        if (sotmEnabled) {
            X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
            Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

            double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
            double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
            double TOF = Shooter.INSTANCE.getTOF(goalDistance * 0.0254, Math.toRadians(hoodAngle));

            Configuration.setAimPointOffset(
                    -X_VELOCITY * (Configuration.ARTIFACT_TRANSFER_TIME + TOF),
                    -Y_VELOCITY * (Configuration.ARTIFACT_TRANSFER_TIME + TOF)
            );
        } else {
            X_VELOCITY = 0;
            Y_VELOCITY = 0;
            Configuration.setAimPointOffset(0, 0);
        }

        // Loop timing
        telemetry.addData("Loop (hz)", (int)(1000 / LOOP_TIME));

        // SOTM Status
        telemetry.addLine();
        telemetry.addData("SOTM", sotmEnabled ? "ON" : "OFF");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");

        // Velocity & Offset
        telemetry.addLine();
        telemetry.addData("Velocity (X, Y)", String.format("%.2f, %.2f", X_VELOCITY, Y_VELOCITY));
        telemetry.addData("Aim Offset (X, Y)", String.format("%.2f, %.2f", Configuration.X_GOAL_OFFSET, Configuration.Y_GOAL_OFFSET));

        // Goal & TOF
        telemetry.addLine();
        telemetry.addData("Distance (in)", String.format("%.1f", Shooter.INSTANCE.GOAL_DISTANCE));
        telemetry.addData("TOF (sec)", String.format("%.3f", Shooter.INSTANCE.getTOF(Shooter.INSTANCE.GOAL_DISTANCE * 0.0254, Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE))));

        // Shooter
        telemetry.addLine();
        telemetry.addData("Hood Angle", String.format("%.1f°", Shooter.INSTANCE.HOOD_ANGLE));
        telemetry.addData("Target RPM", (int) Shooter.INSTANCE.FLYWHEEL_RPM_GOAL);
        double avgRPM = (Shooter.velocityToRPM(Math.abs(Shooter.INSTANCE.flywheelMotor1.getMotor().getVelocity())) +
                        Shooter.velocityToRPM(Math.abs(Shooter.INSTANCE.flywheelMotor2.getMotor().getVelocity()))) / 2;
        telemetry.addData("Actual RPM", (int) avgRPM);

        // Turret
        telemetry.addLine();
        telemetry.addData("Turret Angle", String.format("%.1f°", Turret.INSTANCE.TURRET_ANGLE));

        // Controls
        telemetry.addLine();
        telemetry.addData("X", "SOTM On/Off");
        telemetry.addData("LB/RB", "Gate");
        telemetry.addData("A", "Intake");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
