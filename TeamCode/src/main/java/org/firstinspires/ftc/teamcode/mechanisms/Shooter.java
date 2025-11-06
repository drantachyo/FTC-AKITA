package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;

public class Shooter {
    private DcMotor shooterMotor;
    private Servo gateServo;

    private final double OPEN_POSITION = 0.65;
    private final double CLOSE_POSITION = 1.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "Outtake");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gateServo = hardwareMap.get(Servo.class, "GateServo");
    }

    public Action runShooter() {
        return new InstantAction(() -> shooterMotor.setPower(0.7));
    }

    public Action stopShooter() {
        return new InstantAction(() -> shooterMotor.setPower(0));
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void openGate() {
        gateServo.setPosition(OPEN_POSITION);
    }

    public void closeGate() {
        gateServo.setPosition(CLOSE_POSITION);
    }
}
