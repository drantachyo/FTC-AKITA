package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Action runIntake() {
        return new InstantAction(() -> intakeMotor.setPower(1.0));
    }

    public Action stopIntake() {
        return new InstantAction(() -> intakeMotor.setPower(0));
    }

    public void setPower(double v) {
        intakeMotor.setPower(v);
    }
}
