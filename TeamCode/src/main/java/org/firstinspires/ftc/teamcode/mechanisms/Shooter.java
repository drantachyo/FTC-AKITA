package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    private final DcMotorEx shooterMotor;

    private double targetRPM = 0.0;
    private double lastRPM = 0.0;

    private static final double TPR = 28.0;
    private static final double MAX_RPM = 6000.0;
    private static final double RPM_TOLERANCE = 70.0;

    private final PIDFCoefficients pidf =
            new PIDFCoefficients(33.0, 0, 2.0, 14.0);

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void update() {
        double targetTPS = (targetRPM * TPR) / 60.0;

        if (targetRPM > 0)
            shooterMotor.setVelocity(targetTPS);
        else
            shooterMotor.setPower(0);

        lastRPM = (shooterMotor.getVelocity() * 60.0) / TPR;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm; // сохраняем цель
        double targetTPS = targetRPM / 60.0 * 28.0; // переводим в такты на секунду
        shooterMotor.setVelocity(targetTPS);
    }



    public boolean isStable() {
        return Math.abs(lastRPM - targetRPM) <= RPM_TOLERANCE;
    }

    public DcMotorEx getMotor() {
        return shooterMotor;
    }

}
