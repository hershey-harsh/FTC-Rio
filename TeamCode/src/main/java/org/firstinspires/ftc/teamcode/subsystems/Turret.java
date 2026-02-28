package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configuration;

//import static org.firstinspires.ftc.teamcode.subsystems.Light.GREEN;
//import static org.firstinspires.ftc.teamcode.subsystems.Light.YELLOW;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    public static double ANGLE_INCREMENT = 1.0;

    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public ServoGroup turretServo;

    public double HEADING_DEGREE = 0, TARGET_DEGREE = 0, ODO_TARGET = 0, TURRET_POSITION = 0, TURRET_ANGLE = 0, ERROR = 0;
    public double MANUAL_ANGLE = 0; // Stores the manual angle offset

    public Mode mode = Mode.odometry;

    public enum Mode {
        manual,
        odometry,
    }

    private Turret() {}

    @Override
    public void initialize() {
        turretServo1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.TURRET_SERVO_LEFT));
        turretServo2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.TURRET_SERVO_RIGHT));
//        turretServo2.getServo().setDirection(Servo.Direction.REVERSE);
        turretServo = new ServoGroup(turretServo1, turretServo2);

        // Enable PWM so servos actually move
        turretServo1.getServo().getController().pwmEnable();
        turretServo2.getServo().getController().pwmEnable();

        setAngle(0);
    }

    @Override
    public void periodic() {
        Pose pose = Configuration.CURRENT_POSE;
        HEADING_DEGREE = Math.toDegrees(pose.getHeading());

        if (Configuration.ALLIANCE == Configuration.Alliance.RED) {
            ODO_TARGET = Math.atan(
                    ((Configuration.RED_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY())/
                    ((Configuration.RED_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX())
            );
        } else {
            ODO_TARGET = Math.atan2(
                    ((Configuration.BLUE_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY()),
                    ((Configuration.BLUE_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX())
            );
        }

        if (mode == Mode.odometry) {
            // Calculate turret angle: angle to goal minus robot heading
            // Normalize to get the shortest path
            TARGET_DEGREE = Math.toDegrees(ODO_TARGET) - HEADING_DEGREE;
            setAngle(TARGET_DEGREE);
        } else if (mode == Mode.manual) {
            // In manual mode, use the stored MANUAL_ANGLE
            setAngle(TARGET_DEGREE);
        }
    }

    private void setAngle(double angle) {
//        double angle;
//        angle = AngleUnit.normalizeDegrees(target_angle + Configuration.TURRET_OFFSET);
        TURRET_ANGLE = angle + Configuration.TURRET_OFFSET;
//
//        boolean unreachable = false;
//        if (angle > 165) {
//            angle = 165;
//            unreachable = true;
//        } else if (angle < -155) {
//            angle = -155;
//            unreachable = true;
//        }
//
////        if (unreachable) {
////            Light.INSTANCE.setColor(YELLOW, Light.Target.TURRET).schedule();
////        } else {
////            Light.INSTANCE.setColor(GREEN, Light.Target.TURRET).schedule();
////        }
//
//        TURRET_POSITION = (angle + 155) / 320;

        TURRET_POSITION = interpolateAngle(angle);

        turretServo.setPosition(TURRET_POSITION);
//        turretServo2.setPosition(TURRET_POSITION);
    }

    private double interpolateAngle(double angle) {
        angle = AngleUnit.normalizeDegrees(angle);
        if (angle > 165) {
            angle = 165;
        } else if (angle < -155) {
            angle = -155;
        }

//        double result = 0.5 - (angle) / 150;

        double result = 0.5 - (angle * 0.25) / 90;

        if (result > 0.750) {
            result = 0.750;
        } else if (result < 0.246) {
            result = 0.246;
        }

        return result;
    }

    public Command start() {
        return new InstantCommand(() -> {
            turretServo1.getServo().getController().pwmEnable();
            turretServo2.getServo().getController().pwmEnable();
            setAngle(0);
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            turretServo1.getServo().getController().pwmDisable();
            turretServo2.getServo().getController().pwmDisable();
        });
    }

    public Command changeToManual() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            MANUAL_ANGLE = 0; // Reset manual angle to 0 when switching to manual
        });
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> {
            mode = Mode.odometry;
        });
    }

    public Command increaseAngle() {
        return new InstantCommand(() -> {
            MANUAL_ANGLE = MANUAL_ANGLE + ANGLE_INCREMENT;
            // Clamp to valid range
            if (MANUAL_ANGLE > 165) {
                MANUAL_ANGLE = 165;
            } else if (MANUAL_ANGLE < -155) {
                MANUAL_ANGLE = -155;
            }
        });
    }

    public Command decreaseAngle() {
        return new InstantCommand(() -> {
            MANUAL_ANGLE = MANUAL_ANGLE - ANGLE_INCREMENT;
            // Clamp to valid range
            if (MANUAL_ANGLE > 165) {
                MANUAL_ANGLE = 165;
            } else if (MANUAL_ANGLE < -155) {
                MANUAL_ANGLE = -155;
            }
        });
    }

}
