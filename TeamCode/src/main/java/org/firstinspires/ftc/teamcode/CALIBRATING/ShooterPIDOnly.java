package org.firstinspires.ftc.teamcode.CALIBRATING;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter PID Only", group = "TeleOp")
public class ShooterPIDOnly extends OpMode {

    DcMotorEx shooter;

    double kP = 20.0, kI = 0, kD = 2.0, kF = 14.0;

    double targetRPM = 3000;
    double targetTPS;

    // boost logic
    double boostPower = 0.15;
    long boostTime = 120;
    long boostStart = 0;
    double dropThreshold = 250;

    // buttons
    boolean shooterOn = false;
    boolean lastX = false;
    boolean lastA = false;
    boolean lastLB = false;
    boolean lastRB = false;

    boolean reversed = false;

    @Override
    public void init() {

        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        targetTPS = targetRPM / 60.0 * 28.0;

        telemetry.addLine("Shooter PID Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean curX = gamepad2.x;
        boolean curA = gamepad2.a;
        boolean curLB = gamepad2.left_bumper;
        boolean curRB = gamepad2.right_bumper;

        // toggle shooter on/off
        if(curX && !lastX) shooterOn = !shooterOn;

        // reverse direction
        if(curA && !lastA){
            reversed = !reversed;
            shooter.setDirection(reversed ?
                    DcMotorSimple.Direction.FORWARD :
                    DcMotorSimple.Direction.REVERSE);
        }

        // adjust RPM
        if(curLB && !lastLB) targetRPM = Math.max(500, targetRPM - 250);
        if(curRB && !lastRB) targetRPM = Math.min(6000, targetRPM + 250);

        targetTPS = targetRPM / 60.0 * 28.0;

        double currentRPM = shooter.getVelocity() / 28.0 * 60.0;

        if(shooterOn){
            if(targetRPM - currentRPM > dropThreshold)
                boostStart = System.currentTimeMillis();

            boolean boosting = (System.currentTimeMillis() - boostStart) < boostTime;

            double velocity = targetTPS * (boosting ? 1.0 + boostPower : 1.0);
            shooter.setVelocity(velocity);
        } else {
            shooter.setPower(0);
        }

        // update last buttons
        lastX = curX;
        lastA = curA;
        lastLB = curLB;
        lastRB = curRB;

        // telemetry
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Direction", reversed ? "FORWARD" : "REVERSE");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }
}
