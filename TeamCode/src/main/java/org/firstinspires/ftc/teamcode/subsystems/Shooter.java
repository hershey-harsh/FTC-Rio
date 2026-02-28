package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

import org.firstinspires.ftc.teamcode.Configuration;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;


public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static final double TICKS_PER_REV = 28.0;
    public static final double GEAR_RATIO = 1.0; // Direct drive (motor RPM = flywheel RPM)

    public static double HOOD_INCREMENT = 0.05;
    public static double RPM_INCREMENT = 100.0;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.0125, 0.0, 0.0001);
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(0.0, 0.0, 0.0);

    public MotorEx flywheelMotor1;
    public MotorEx flywheelMotor2;
    public MotorGroup flywheelMotor;

    public ServoEx hoodServo1;
    public ServoEx hoodServo2;
    public ServoGroup hoodServo;

    private ControlSystem controlSystem;

    public double FLYWHEEL_RPM;
    public double FLYWHEEL_RPM_GOAL = 0;
    private double FLYWHEEL_RPM_INTOLERANCE = 100.0;

    private final double ratio = 32.0/340.0, range = 355;
    public double HOOD_ANGLE = 70;
    public double HOOD_POSITION = 0.1;
    public double MIN_HOOD_ANGLE = 43.0;
    public double MAX_HOOD_ANGLE = 70.0;

    public Mode mode = Mode.odometry;

    // Flywheel radius in meters (for velocity calculation)
    private static final double FLYWHEEL_RADIUS_METERS = 0.036;

    public double GOAL_DISTANCE = 0;

    public enum Mode {
        manual,
        odometry,
    }

    @Override
    public void initialize() {
        flywheelMotor1 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.RIGHT_TURRET_MOTOR));
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor1.getMotor().setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotor2 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.LEFT_TURRET_MOTOR));
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.getMotor().setDirection(DcMotorEx.Direction.FORWARD);

        flywheelMotor = new MotorGroup(flywheelMotor1, flywheelMotor2);

        hoodServo1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.HOOD_SERVO_LEFT));
        hoodServo2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.HOOD_SERVO_RIGHT));
        hoodServo2.getServo().setDirection(Servo.Direction.REVERSE);
        hoodServo = new ServoGroup(hoodServo1, hoodServo2);

        controlSystem = ControlSystem.builder()
                .velPid(coefficients)
                .basicFF(ffcoefficients)
                .build();
    }

    @Override
    public void periodic() {
        Pose pose = Configuration.CURRENT_POSE;

        if (Configuration.ALLIANCE == Configuration.ALLIANCE.RED) {
            GOAL_DISTANCE = Math.hypot(
                    (Configuration.RED_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.RED_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY());
        } else {
            GOAL_DISTANCE = Math.hypot(
                    (Configuration.BLUE_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.BLUE_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY());
        }

        if (mode == Shooter.Mode.odometry) {
            // Use math model to calculate RPM based on current hood position
            double distMeters = GOAL_DISTANCE * 0.0254;
            FLYWHEEL_RPM_GOAL = fetchRPM(distMeters, HOOD_ANGLE);
            setTargetRPM(FLYWHEEL_RPM_GOAL, FLYWHEEL_RPM_INTOLERANCE);
        }

        //TODO: Add manual mode.

        flywheelMotor.setPower(controlSystem.calculate(flywheelMotor1.getState()));
    }

    public void setTargetRPM(double FLYWHEEL_RPM, double FLYWHEEL_RPM_INTOLERANCE){
        this.FLYWHEEL_RPM = FLYWHEEL_RPM;
        this.FLYWHEEL_RPM_INTOLERANCE = FLYWHEEL_RPM_INTOLERANCE;

        double motorVelocity = rpmToVelocity(FLYWHEEL_RPM);
        double toleranceVelocity = rpmToVelocity(FLYWHEEL_RPM_INTOLERANCE);

        new RunToVelocity(controlSystem, motorVelocity, toleranceVelocity).schedule();
    }

    public static double velocityToRPM(double ticksPerSec) { return ticksPerSec * 60.0 / TICKS_PER_REV; }

    public static double rpmToVelocity(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public void setHoodAngle(double degrees) {
        // Clamp degrees to valid range (43 to 70)
        if (degrees >= 70) {
            degrees = 70;
        } else if (degrees <= 43) {
            degrees = 43;
        }

        HOOD_ANGLE = degrees;

        HOOD_POSITION = 0.1 + (70 - degrees) * (0.9 / 27.0);
        hoodServo.setPosition(HOOD_POSITION);
    }

    private double a = 0, b = 0, c = 0, n = 0, t_u = 0, t_g = 0, tof = 0, vX = 0, vY = 0, v = 0, m = 0;

    public double getHoodAngle(double meters) {
        return Math.max((-4.8701*meters) + 59.754, 42);
    }

    public double fetchRPM(double distMeters, double hoodAngle) {
        a = (-distMeters * Math.tan(Math.toRadians(hoodAngle)) + Configuration.SHOOTER_HEIGHT_TO_GOAL) / (distMeters * distMeters);
        b = -Math.tan(Math.toRadians(hoodAngle))-(2*a*distMeters);
        n = -b/(2*a);
        m = (a * (n*n)) + (b * (n)) + Configuration.SHOOTER_HEIGHT_TO_GOAL;

        t_u = Math.sqrt((2*m) / 9.8);
        t_g = Math.sqrt((2*(m-Configuration.SHOOTER_HEIGHT_TO_GOAL)) / 9.8);
        tof = t_u + t_g;

        vX = distMeters/tof;
        vY = (m - 0.5*(-9.8)*(t_u*t_u))/t_u;

        v = Math.sqrt((vX*vX) + (vY*vY));
        if (GOAL_DISTANCE * 0.0254 > 4) {
            return (v / (2*Math.PI * 0.036)) * 60 * 3;
        }
        return (v / (2*Math.PI * 0.036)) * 60 * 2.2;
    }

    /// 97.827, 47.5
    /// 54.367, 58.5
    /// 102.06, 52
    /// 78.87, 48

    public double getTOF(double distMeters, double hoodAngleRad) {
        a = (-distMeters * Math.tan(hoodAngleRad) + Configuration.SHOOTER_HEIGHT_TO_GOAL) / (distMeters * distMeters);
        b = -Math.tan(hoodAngleRad)-(2*a*distMeters);
        n = -b/(2*a);
        m = (a * (n*n)) + (b * (n)) + Configuration.SHOOTER_HEIGHT_TO_GOAL;

        t_u = Math.sqrt((2*m) / 9.8);
        t_g = Math.sqrt((2*(m-Configuration.SHOOTER_HEIGHT_TO_GOAL)) / 9.8);
        tof = t_u + t_g;

        return tof;
    }

    public Command changeToManual() {
        return new InstantCommand(() -> {
            mode = Shooter.Mode.manual;
            setTargetRPM(3800, FLYWHEEL_RPM_INTOLERANCE);
        });
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> {
            mode = Shooter.Mode.odometry;
        });
    }

    public Command on() {
        return new InstantCommand(() -> {
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
            mode = Shooter.Mode.odometry;
        });
    }

    public Command off() {
        return new InstantCommand(() -> {
            mode = Shooter.Mode.manual;
            setTargetRPM(0, FLYWHEEL_RPM_INTOLERANCE);
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            mode = Shooter.Mode.manual;
            hoodServo1.getServo().getController().pwmEnable();
            hoodServo2.getServo().getController().pwmEnable();
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
        });
    }
    public Command emergencyStop() {
        return new InstantCommand(() -> {
            mode = Shooter.Mode.manual;
            hoodServo1.getServo().getController().pwmDisable();
            hoodServo2.getServo().getController().pwmDisable();
            flywheelMotor1.getMotor().setMotorDisable();
            flywheelMotor2.getMotor().setMotorDisable();
        });
    }

    public Command increaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = HOOD_ANGLE + HOOD_INCREMENT;
            if (newAngle > MAX_HOOD_ANGLE) {
                newAngle = MAX_HOOD_ANGLE;
            }
            HOOD_ANGLE = newAngle;
            setHoodAngle(HOOD_ANGLE);
        });
    }

    public Command decreaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = HOOD_ANGLE - HOOD_INCREMENT;
            if (newAngle < MIN_HOOD_ANGLE) {
                newAngle = MIN_HOOD_ANGLE;
            }
            HOOD_ANGLE = newAngle;
            setHoodAngle(HOOD_ANGLE);
        });
    }

    public Command adjustShooterRPM(double stickValue) {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newRPM = FLYWHEEL_RPM + (stickValue * RPM_INCREMENT);
            if (newRPM < 0) {
                newRPM = 0;
            }
            setTargetRPM(newRPM, FLYWHEEL_RPM_INTOLERANCE);
        });
    }

}