package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Configuration;

import static org.firstinspires.ftc.teamcode.subsystems.Light.AZURE;
import static org.firstinspires.ftc.teamcode.subsystems.Light.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Light.YELLOW;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    public LLResult limelightResult;
    private int pipeline;
    public boolean autoUpdateEnabled = true;

    private Limelight() {}

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, Configuration.LIMELIGHT);

        if (Configuration.ALLIANCE == Configuration.ALLIANCE.RED) {
            pipeline = Configuration.RED_LIMELIGHT_PIPELINE;
        } else {
            pipeline = Configuration.BLUE_LIMELIGHT_PIPELINE;
        }

        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Override
    public void periodic() {
        if (autoUpdateEnabled) {
            Light.INSTANCE.setBlinkingColor(AZURE, 500, Light.Target.ROBOT).schedule();
            updateLocalization();
        }
    }

    private void updateLocalization() {
        limelightResult = limelight.getLatestResult();

        if (limelightResult != null && limelightResult.isValid()) {

            Pose3D botpose3D = limelightResult.getBotpose();
            if (botpose3D != null) {
                Pose2D botpose2D = new Pose2D(
                        DistanceUnit.METER,
                        botpose3D.getPosition().x,
                        botpose3D.getPosition().y,
                        AngleUnit.DEGREES,
                        botpose3D.getOrientation().getYaw()
                );

                Pose ftcPose = PoseConverter.pose2DToPose(botpose2D, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                Configuration.fusionLocalizer.addMeasurement(pedroPose, System.nanoTime() - limelightResult.getStaleness());

                if (!autoUpdateEnabled) {
                    Light.INSTANCE.setColor(BLUE, Light.Target.ROBOT).schedule();
                }
            }
        }
    }

    public Command start() {
        return new InstantCommand(() -> limelight.start());
    }

    public Command stop() {
        return new InstantCommand(() -> limelight.stop());
    }

    public Command enableAutoUpdate() {
        return new InstantCommand(() -> autoUpdateEnabled = true);
    }

    public Command disableAutoUpdate() {
        Light.INSTANCE.setColor(BLUE, Light.Target.ROBOT).schedule();
        return new InstantCommand(() -> autoUpdateEnabled = false);
    }

    public Command update() {
        return new InstantCommand(() -> {
            if (autoUpdateEnabled) return;
            Light.INSTANCE.setColor(AZURE, Light.Target.ROBOT).schedule();
            updateLocalization();
        });
    }

    public Command pause() {
        return new InstantCommand(() -> limelight.pause());
    }
}
