package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Configuration;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static final double TICKS_PER_REV = 28.0;

//    public static double kP = 0.001, kI = 0, kD = 0, kF = 0.00015;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.0125, 0.0, 0.0001);
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(0.0, 0.0, 0.0);
//    private PIDController velController;
//    private VoltageSensor voltageSensor;

    public static double HOOD_INCREMENT = 0.05;
    public static double RPM_INCREMENT = 100.0;

    public MotorEx flywheelMotor1;
    public MotorEx flywheelMotor2;
    public MotorGroup flywheelMotor;

    private ControlSystem controlSystem;

    public ServoEx hoodServo1;
    public ServoEx hoodServo2;
    public ServoGroup hoodServo;

    public double FLYWHEEL_RPM_GOAL = 0;

    public double HOOD_ANGLE = 70;
    public double HOOD_POSITION = 0.1;
    public double MIN_HOOD_ANGLE = 43.0;
    public double MAX_HOOD_ANGLE = 70.0;

    public Mode mode = Mode.odometry;

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

//        velController = new PIDController(kP, kI, kD);
//        voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        Pose pose = Configuration.CURRENT_POSE;
        flywheelMotor.setPower(controlSystem.calculate(flywheelMotor.getState()));
        
        if (Configuration.ALLIANCE == Configuration.Alliance.RED) {
            GOAL_DISTANCE = Math.hypot(
                    (Configuration.RED_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.RED_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY());
        } else {
            GOAL_DISTANCE = Math.hypot(
                    (Configuration.BLUE_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.BLUE_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY());
        }

        // Always update readRPM so telemetry is fresh regardless of shooter mode
        // Motor1 is set REVERSE so raw velocity is already negative when spinning forward
        double encoderVelocity = flywheelMotor1.getState().getVelocity();
        readRPM = Math.abs(((encoderVelocity * 60.0) / TICKS_PER_REV) * (16.0 / 16.0));

        // Hood angle is computed here; SOTM kinematics and RPM are handled in onUpdate
        double distMeters = GOAL_DISTANCE * 0.0254;
        HOOD_ANGLE = getHoodAngle(distMeters);
        if (mode == Mode.odometry) {
            setHoodAngle(HOOD_ANGLE);
        }

        // Update velocity setpoint directly every loop instead of scheduling a command
        double targetVelocity = rpmToVelocity(targetRPM);
        controlSystem.setGoal(new KineticState(0.0, targetVelocity));
    }

    public void setHoodAngle(double degrees) {
        if (degrees >= 70) {
            degrees = 70;
        } else if (degrees <= 43) {
            degrees = 43;
        }
        HOOD_ANGLE = degrees;
        HOOD_POSITION = 0.1 + (70 - degrees) * (0.9 / 27.0);
        hoodServo.setPosition(HOOD_POSITION);
    }

    public double getHoodAngle(double meters) {
        return Math.max((-4.8701 * meters) + 59.754, 43);
    }

    private double a, b, n, t_u, t_g, tof, vX, vY, v, m;
    private double kinematicRPMGoal = 0;

    public static double w = Configuration.SHOOTER_HEIGHT_TO_GOAL;
    public static double vcWeight = 0.37;
    public static double RPM_OFFSET = 0;
    public static double FAR_DISTANCE_THRESHOLD = 3.0; // meters
    public static double FAR_RPM = 4400;
    public static double FAR_HOOD_ANGLE = 50;

    public void updateKinematics(double distMeters, double hoodAngleRad) {
        a = (-distMeters * Math.tan(hoodAngleRad) + w) / (distMeters * distMeters);
        b = -Math.tan(hoodAngleRad) - (2 * a * distMeters);
        n = -b / (2 * a);
        m = (a * (n * n)) + (b * n) + w;

        t_u = Math.sqrt((2 * m) / 9.8);
        t_g = Math.sqrt((2 * (m - w)) / 9.8);
        tof = t_u + t_g;

        vX = distMeters / tof;
        vY = (m - 0.5 * (-9.8) * (t_u * t_u)) / t_u;

        v = Math.sqrt((vX * vX) + (vY * vY)) * 1.1;

        kinematicRPMGoal = (v / (2 * Math.PI * 0.036)) * 60;
    }

    public double shooterVKinematic() { return v; }
    public double getTof() { return tof; }
    public double getKinematicRPMGoal() { return kinematicRPMGoal; }

    public double vMSToRPM(double vMS) {
        return (vMS / (2 * Math.PI * 0.036)) * 60;
    }

    public double targetRPM = 0;
    public double readRPM = 0;
    public double runMs = 0;

    private final ElapsedTime functionRunLength = new ElapsedTime();

    public static double rpmToVelocity(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public void runShooterClose() {
        // setpoint is updated in periodic()
    }

    public void runShooterFar() {
        functionRunLength.reset();

//        setHoodAngle(FAR_HOOD_ANGLE);
//        targetRPM = FAR_RPM + RPM_OFFSET;
//
//        if (readRPM < targetRPM) {
//            flywheelMotor1.getMotor().setPower(1);
//            flywheelMotor2.getMotor().setPower(1);
//        } else {
//            flywheelMotor1.getMotor().setPower(0);
//            flywheelMotor2.getMotor().setPower(0);
//        }
//
//        runMs = functionRunLength.milliseconds();
    }

//    public void runShooter() {
//        functionRunLength.reset();
//
//        double encoderVelocity = -flywheelMotor1.getState().getVelocity();
//        double rpm = ((encoderVelocity * 60.0) / TICKS_PER_REV) * (55.0 / 65.0);
//        readRPM = rpm;
//
//        double power = velController.calculate(rpm, targetRPM) + (kF * targetRPM);
//        double scalar = 13.2 / voltageSensor.getVoltage();
//
//        flywheelMotor1.getMotor().setPower(power * scalar);
//        flywheelMotor2.getMotor().setPower(power * scalar);
//
//        runMs = functionRunLength.milliseconds();
//    }

    public void stopShooter() {
        flywheelMotor1.getMotor().setPower(0);
        flywheelMotor2.getMotor().setPower(0);
    }

    public double getWeight() {
        if (!Double.isNaN(tof)) {
            return tof + Configuration.ARTIFACT_TRANSFER_TIME;
        }
        return 0.3;
    }

    public Command on() {
        return new InstantCommand(() -> {
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
            mode = Mode.odometry;
        });
    }

    public Command off() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            stopShooter();
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            hoodServo1.getServo().getController().pwmEnable();
            hoodServo2.getServo().getController().pwmEnable();
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            hoodServo1.getServo().getController().pwmDisable();
            hoodServo2.getServo().getController().pwmDisable();
            flywheelMotor1.getMotor().setMotorDisable();
            flywheelMotor2.getMotor().setMotorDisable();
        });
    }

    public Command changeToManual() {
        return new InstantCommand(() -> mode = Mode.manual);
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> mode = Mode.odometry);
    }

    public Command increaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = Math.min(HOOD_ANGLE + HOOD_INCREMENT, MAX_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command decreaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = Math.max(HOOD_ANGLE - HOOD_INCREMENT, MIN_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command adjustShooterRPM(double stickValue) {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            targetRPM = Math.max(0, targetRPM + (stickValue * RPM_INCREMENT));
        });
    }
}

