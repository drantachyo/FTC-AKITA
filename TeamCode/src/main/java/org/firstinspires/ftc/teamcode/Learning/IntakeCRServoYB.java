package org.firstinspires.ftc.teamcode.Learning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake + CRServo Y/B Control", group = "TeleOp")
public class IntakeCRServoYB extends OpMode {
    DcMotor Intake;
    CRServo feederServo;


    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        feederServo = hardwareMap.get(CRServo.class, "GateServo");
        feederServo.setPower(0);

        telemetry.addLine("Intake + CRServo Y/B Control Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Intake всегда включен вперед
        if(gamepad2.y){
            Intake.setPower(1.0);
            feederServo.setPower(-1.0);  // Feeder против подачи
        } else if(gamepad2.b){
            Intake.setPower(1.0);
            feederServo.setPower(1.0);   // Feeder по подаче
        } else {
            Intake.setPower(0);
            feederServo.setPower(0);
        }

        telemetry.addData("Intake Power", Intake.getPower());
        telemetry.addData("Feeder Power", feederServo.getPower());
        telemetry.update();
    }


}
