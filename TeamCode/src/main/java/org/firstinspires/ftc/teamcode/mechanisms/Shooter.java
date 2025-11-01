package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;

public class Shooter {
    private DcMotor shooterMotor;
    private Servo feeder;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "Outtake"); // название из конфигурации
        feeder = hardwareMap.get(Servo.class, "Feeder");           // название серво
    }

    public Action runShooter() {
        return new InstantAction(() -> shooterMotor.setPower(0.7));
    }

    public Action stopShooter() {
        return new InstantAction(() -> shooterMotor.setPower(0));
    }

    public Action feed() {
        return new InstantAction(() -> {
            feeder.setPosition(1.0); // подаём шарик
        });
    }

    public Action resetFeeder() {
        return new InstantAction(() -> {
            feeder.setPosition(0.0); // возвращаем обратно
        });
    }
}
