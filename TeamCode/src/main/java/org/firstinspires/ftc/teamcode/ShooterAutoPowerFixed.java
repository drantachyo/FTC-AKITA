package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Shooter Auto Power (Dynamic Range)", group = "Test")
public class ShooterAutoPowerFixed extends LinearOpMode {

    private DcMotor outtake;
    private AprilTagWebcam camera;
    private Double lastKnownDistance = null;

    private boolean shooterOn = false;
    private boolean aWasPressed = false;
    private double lastPowerCoef = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);

        telemetry.addLine("Press PLAY to start camera initialization...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        camera = new AprilTagWebcam();
        camera.init(hardwareMap, telemetry);

        telemetry.addLine("Camera initialized.");
        telemetry.update();

        sleep(1000);

        while (opModeIsActive()) {
            camera.update();
            AprilTagDetection tag = camera.getTagBySpecificId(20);

            if (tag != null && tag.ftcPose != null) {
                lastKnownDistance = tag.ftcPose.range;
                telemetry.addData("✅ Tag ID", tag.id);
                telemetry.addData("Distance (cm)", "%.1f", tag.ftcPose.range);
            } else {
                telemetry.addLine("❌ Tag not visible");
            }


            // тумблер
            if (gamepad1.a && !aWasPressed) {
                shooterOn = !shooterOn;
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            if (shooterOn) {
                if (lastKnownDistance != null) {
                    double power = calculateShooterPower(lastKnownDistance);
                    outtake.setPower(1.0);
                    lastPowerCoef = power;

                    telemetry.addData("Shooter Power", "%.2f", power);
                    telemetry.addData("Power Coefficient", "%.2f", lastPowerCoef);
                } else {
                    outtake.setPower(0);
                    telemetry.addLine("No tag data — shooter off");
                }
            } else {
                outtake.setPower(0);
            }

            telemetry.addData("Shooter State", shooterOn ? "ON" : "OFF");
            telemetry.update();
        }

        camera.stop();
    }

    // кривая мощности по дистанции (примерно 0.4 при 90 см → 1.0 при 300 см)
    private double calculateShooterPower(double distanceCm) {
        double minDist = 120;   // ближняя дистанция
        double maxDist = 300;  // дальняя дистанция
        double minPower = 0.45;
        double maxPower = 1.0;

        distanceCm = Math.max(minDist, Math.min(maxDist, distanceCm));

        // нормализуем расстояние
        double norm = (distanceCm - minDist) / (maxDist - minDist);

        // нелинейная кривая (1.6 — экспериментально, можно менять)
        double curve = Math.pow(norm, 2.15);

        return minPower + curve * (maxPower - minPower);
    }

}
