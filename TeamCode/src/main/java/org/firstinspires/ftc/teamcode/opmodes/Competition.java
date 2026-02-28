package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
//import org.firstinspires.ftc.teamcode.subsystems.Light;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TeleOp")
public class Competition extends NextFTCOpMode {
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;
    private boolean gamepad2Override = false;

    public Competition() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
//                new SubsystemComponent(Light.INSTANCE),
//                new SubsystemComponent(Limelight.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        PedroComponent.follower().setStartingPose(Configuration.CURRENT_POSE);
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();

//        button(() -> gamepad1.a && !gamepad2Override)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> Limelight.INSTANCE.enableAutoUpdate().schedule())
//                .whenBecomesFalse(() -> Limelight.INSTANCE.disableAutoUpdate().schedule());

//        button(() -> gamepad1.b && !gamepad2Override)
//                .whenTrue(() -> Limelight.INSTANCE.update().schedule());

        button(() -> gamepad1.x && !gamepad2Override)
                .whenTrue(() -> PedroComponent.follower().setPose(Configuration.MANUAL_LOCALIZATION_POSE));

        button(() -> gamepad1.back && !gamepad2Override)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
//                    Light.INSTANCE.setBlinkingColor(Light.RED, 250).schedule();
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStop().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
//                    Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        ///  Gamepad 2 Bindings  ///

//        button(() -> gamepad2.right_bumper)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> Turret.INSTANCE.changeToAuto())
//                .whenBecomesFalse(() -> Turret.INSTANCE.changeToManual());
///        GATE OPEN/CLOSE

        button(() -> gamepad2.left_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Shooter.INSTANCE.on().schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.off().schedule());

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        button(() -> gamepad2.b)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.gate3Start().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.gate3Stop().schedule());

        button(() -> gamepad2.a)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.gate1And2Start().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.gate1And2Stop().schedule());

        button(() -> gamepad2.y)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Shooter.INSTANCE.changeToAuto().schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.changeToManual().schedule());

        button(() -> gamepad2.x)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Turret.INSTANCE.changeToAuto().schedule())
                .whenBecomesFalse(() -> Turret.INSTANCE.changeToManual().schedule());

        button(() -> gamepad2.dpad_left)
                .whenTrue(() -> Turret.INSTANCE.increaseAngle().schedule());

        button(() -> gamepad2.dpad_right)
                .whenTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());

        button(() -> gamepad2.dpad_up)
                .whenTrue(() -> Shooter.INSTANCE.increaseHood().schedule());

        button(() -> gamepad2.dpad_down)
                .whenTrue(() -> Shooter.INSTANCE.decreaseHood().schedule());

        button(() -> gamepad2.start)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> {
                    gamepad2Override = true;
                    driverControlled.cancel();
                    driverControlled = new PedroDriverControlled(
                            Gamepads.gamepad2().leftStickY(),
                            Gamepads.gamepad2().leftStickX(),
                            Gamepads.gamepad2().rightStickX(),
                            !Configuration.FIELD_CENTRIC
                    );
                    driverControlled.schedule();
                })
                .whenBecomesFalse(() -> {
                    gamepad2Override = false;
                    driverControlled.cancel();
                    driverControlled = new PedroDriverControlled(
                            Gamepads.gamepad1().leftStickY(),
                            Gamepads.gamepad1().leftStickX(),
                            Gamepads.gamepad1().rightStickX(),
                            !Configuration.FIELD_CENTRIC
                    );
                    driverControlled.schedule();
                });

        button(() -> gamepad2.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
//                    Light.INSTANCE.setBlinkingColor(Light.RED, 250).schedule();
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStop().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
//                    Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        //TODO: Test left stick x on Gamepad 2.

        range(() -> !gamepad2Override ? gamepad2.left_stick_x : 0)
                .deadZone(0.1)
                .asButton(value -> value != 0)
                .whenTrue(() -> Shooter.INSTANCE.adjustShooterRPM(gamepad2.left_stick_x));
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));
        telemetry.addData("Drive Control", gamepad2Override ? "Gamepad 2" : "Gamepad 1");

        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
        double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
        double TOF = Shooter.INSTANCE.getTOF(goalDistance, Math.toRadians(hoodAngle));

        Configuration.setAimPointOffset(-X_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF,
                -Y_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF);

        telemetry.addData("COLOR R:", "%.3f", Transfer.redValues);
        telemetry.addData("COLOR G:", "%.3f", Transfer.greenValues);
        telemetry.addData("COLOR B:", "%.3f", Transfer.blueValues);
        telemetry.addData("Is Purple:", Transfer.INSTANCE.isPurple());
        telemetry.addData("Is Green:", Transfer.INSTANCE.isGreen());
        telemetry.addData("=== Position ===", "");
        telemetry.addData("Position X:", PedroComponent.follower().getPose().getX());
        telemetry.addData("Position Y:", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

//        telemetry.addData("=== Limelight ===", "");
//        telemetry.addData("LL Auto Update", Limelight.INSTANCE.autoUpdateEnabled);
//        telemetry.addData("LL Localizing", Limelight.INSTANCE.limelightResult != null && Limelight.INSTANCE.limelightResult.isValid());

        telemetry.addData("=== Turret ===", "");
        telemetry.addData("Turret Mode", Turret.INSTANCE.mode);
//        telemetry.addData("Turret Angle", Turret.INSTANCE.TURRET_ANGLE);
//        telemetry.addData("Turret Target", Turret.INSTANCE.TARGET_DEGREE);
//        telemetry.addData("Turret Servo Pos", "%.3f", Turret.INSTANCE.TURRET_POSITION);

        telemetry.addData("=== Shooter ===", "");
        telemetry.addData("Shooter Mode", Shooter.INSTANCE.mode);
        telemetry.addData("Goal Distance", Shooter.INSTANCE.GOAL_DISTANCE);
        //telemetry.addData("Flywheel RPM Goal", Shooter.INSTANCE.FLYWHEEL_RPM_GOAL);
        //telemetry.addData("Hood Angle", Shooter.INSTANCE.HOOD_ANGLE);
        //telemetry.addData("Time of Flight", TOF);

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}