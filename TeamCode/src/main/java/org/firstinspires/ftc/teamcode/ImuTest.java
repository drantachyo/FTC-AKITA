package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMU Test", group = "Diagnostics")
public class ImuTest extends LinearOpMode {

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        telemetry.addLine("IMU initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation();
            telemetry.addData("Heading (Z)", angles.firstAngle);
            telemetry.addData("Pitch (Y)", angles.secondAngle);
            telemetry.addData("Roll (X)", angles.thirdAngle);
            telemetry.update();
        }
    }
}
