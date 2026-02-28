package org.firstinspires.ftc.teamcode.opmodes.debug;

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
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name = "Intake", group = "Debug")
public class Intake extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    public Intake() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE)
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
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();

        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        button(() -> gamepad1.right_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.stop().schedule());

        button(() -> gamepad1.dpad_up)
                .whenTrue(() -> Transfer.INSTANCE.gate1And2Start().schedule());

        button(() -> gamepad1.dpad_down)
                .whenTrue(() -> Transfer.INSTANCE.gate1And2Stop().schedule());

        button(() -> gamepad1.dpad_right)
                .whenTrue(() -> Transfer.INSTANCE.gate3Start().schedule());

        button(() -> gamepad1.dpad_left)
                .whenTrue(() -> Transfer.INSTANCE.gate3Stop().schedule());

        button(() -> gamepad1.a)
                .whenTrue(() -> Transfer.INSTANCE.emergencyStop().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Loop timing
        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        // Gate servo telemetry
        telemetry.addLine();
        telemetry.addData("Gate 1 Position", Transfer.INSTANCE.servoGate1.getPosition());
        telemetry.addData("Gate 2 Position", Transfer.INSTANCE.servoGate2.getPosition());

        // Transfer motor telemetry
        telemetry.addLine();
        telemetry.addData("Target RPM", Transfer.INTAKE_POWER);
        telemetry.addData("Gate 1 & 2 Velocity (ticks/s)", Transfer.INSTANCE.transferMotor1.getMotor().getVelocity());
        telemetry.addData("Gate 3 Velocity (ticks/s)", Transfer.INSTANCE.transferMotor2.getMotor().getVelocity());
        telemetry.addData("Gate 1 & 2 RPM", Transfer.INSTANCE.transferMotor1.getMotor().getVelocity() * 60.0 / 28.0);
        telemetry.addData("Gate 3 RPM", Transfer.INSTANCE.transferMotor2.getMotor().getVelocity() * 60.0 / 28.0);
        telemetry.addData("Gate 1 & 2 Power", Transfer.INSTANCE.transferMotor1.getMotor().getPower());
        telemetry.addData("Gate 3 Power", Transfer.INSTANCE.transferMotor2.getMotor().getPower());

        // Controls help
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("Left Bumper", "Toggle Gate Open/Close");
        telemetry.addData("Right Bumper", "Toggle Intake/Stop");
        telemetry.addData("D-Pad Up", "Gate 1 & 2 Start");
        telemetry.addData("D-Pad Down", "Gate 1 & 2 Stop");
        telemetry.addData("D-Pad Right", "Gate 3 Start");
        telemetry.addData("D-Pad Left", "Gate 3 Stop");
        telemetry.addData("A Button", "Emergency Stop");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
