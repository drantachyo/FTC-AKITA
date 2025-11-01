package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum TeleOp RR", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor lf, rf, lr, rr;
    private boolean slowMode = false;
    private boolean prevR1 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "leftFront");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        lr = hardwareMap.get(DcMotor.class, "leftBack");
        rr = hardwareMap.get(DcMotor.class, "rightBack");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {

            // Toggle Slow Mode
            boolean currR1 = gamepad1.right_bumper;
            if (currR1 && !prevR1) slowMode = !slowMode;
            prevR1 = currR1;

            boolean braking = gamepad1.b;

            double lx = 0, ly = 0, turn = 0;

            if (!braking) {
                lx = gamepad1.left_stick_x;
                ly = -gamepad1.left_stick_y; // вперед +1
                turn = gamepad1.right_trigger - gamepad1.left_trigger; // триггеры поворот
            }

            // Нормализация мощности
            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(turn), 1.0);

            // Мультипликатор скорости
            double multiplier = slowMode ? 0.3 : 1.0;

            if (braking) {
                // Полная остановка и BRAKE
                lf.setPower(0);
                lr.setPower(0);
                rf.setPower(0);
                rr.setPower(0);

                lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                // Назначаем моторы с учетом мультипликатора
                lf.setPower(((ly + lx + turn) / max) * multiplier);
                lr.setPower(((ly - lx + turn) / max) * multiplier);
                rf.setPower(((ly - lx - turn) / max) * multiplier);
                rr.setPower(((ly + lx - turn) / max) * multiplier);

                // Возвращаем FLOAT
                lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Telemetry
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("LF", lf.getPower());
            telemetry.addData("RF", rf.getPower());
            telemetry.addData("LR", lr.getPower());
            telemetry.addData("RR", rr.getPower());
            telemetry.update();
        }
    }
}
