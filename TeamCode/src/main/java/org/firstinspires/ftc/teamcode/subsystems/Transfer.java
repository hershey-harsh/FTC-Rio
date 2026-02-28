package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;


public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();

    public static double INTAKE_POWER = 1.0;
    public static double POWER_INCREMENT = 0.1;
    public static double GATE_ONE_OPEN = 0.302, GATE_ONE_CLOSED = 0.387;
    public static double GATE_TWO_OPEN = 0.564, GATE_TWO_CLOSED = 0.487;

    public MotorEx transferMotor1;
    public MotorEx transferMotor2;
    public MotorGroup transferMotor;

    public ServoEx servoGate1;
    public ServoEx servoGate2;
    public ServoGroup servoGate;

    public RevColorSensorV3 gate3Sensor;

    public static double redValues = 0;
    public static double blueValues = 0;
    public static double greenValues = 0;
    public static double red = 0;
    public static double blue = 0;
    public static double green = 0;

    public NormalizedRGBA colors;

    public static boolean override = false;

//    public ServoEx servoGate;

    private double currentPower = 0;

    @Override
    public void initialize() {
        transferMotor1 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_ONE));
        transferMotor2 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_TWO)).reversed();

        servoGate1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVO_GATE_LEFT));
        servoGate2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVER_GATE_RIGHT));

        gate3Sensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, Configuration.GATE_3_SENSOR);

        closeGate();
    }

    @Override
    public void periodic() {
        colors = gate3Sensor.getNormalizedColors();

        redValues = colors.red;
        greenValues = colors.green;
        blueValues = colors.blue;

        if ((isPurple() || isGreen()) && !override) {
            transferMotor2.getMotor().setMotorDisable();
        } else if (!isPurple() && !isGreen()) {
            transferMotor2.getMotor().setMotorEnable();
        }
    }

    public boolean isPurple(){
        return red > redValues && blue > blueValues && green > greenValues;
    }
    public boolean isGreen(){
        return red < redValues && blue < blueValues && green > greenValues;
    }

    public Command intake() {
        return new InstantCommand(() -> {
            currentPower = -INTAKE_POWER;
            transferMotor1.setPower(currentPower);
            transferMotor2.setPower(currentPower);
        });
    }

    public Command stop() {
        return new InstantCommand(() -> {
            currentPower = 0;
            transferMotor1.setPower(currentPower);
            transferMotor2.setPower(currentPower);
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            transferMotor1.getMotor().setMotorEnable();
            transferMotor2.getMotor().setMotorEnable();
        });
    }

    public Command gate1And2Start() {
        return new InstantCommand(() -> {
            transferMotor1.getMotor().setMotorEnable();
        });
    }

    public Command gate1And2Stop() {
        return new InstantCommand(() -> {
            transferMotor1.getMotor().setMotorDisable();
        });
    }
    public Command gate3Start() {
        return new InstantCommand(() -> {
            transferMotor2.getMotor().setMotorEnable();
        });
    }

    public Command gate3Stop() {
        return new InstantCommand(() -> {
            transferMotor2.getMotor().setMotorDisable();
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            currentPower = 0;
            transferMotor1.getMotor().setMotorDisable();
            transferMotor2.getMotor().setMotorDisable();
        });
    }

    public Command outtake() {
        return new InstantCommand(() -> {
            transferMotor1.setPower(1);
            transferMotor2.setPower(1);
        });
    }

    public Command increasePower() {
        return new InstantCommand(() -> {
            INTAKE_POWER = Math.min(1.0, INTAKE_POWER + POWER_INCREMENT);
        });
    }

    public Command decreasePower() {
        return new InstantCommand(() -> {
            INTAKE_POWER = Math.max(0, INTAKE_POWER - POWER_INCREMENT);
        });
    }

    public  Command openGate() {
        return new InstantCommand(() -> {
            servoGate1.setPosition(GATE_ONE_OPEN);
            servoGate2.setPosition(GATE_TWO_OPEN);
        });
    }

    public Command closeGate() {
        return new InstantCommand(() -> {
            servoGate1.setPosition(GATE_ONE_CLOSED);
            servoGate2.setPosition(GATE_TWO_CLOSED);
        });
    }
}