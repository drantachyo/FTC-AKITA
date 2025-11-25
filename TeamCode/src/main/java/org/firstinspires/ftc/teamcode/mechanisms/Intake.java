package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intakeMotor;     // Сам интейк
    private final CRServo blockerServo;    // Это теперь и стоппер, и подача


    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        blockerServo = hardwareMap.get(CRServo.class, "GateServo");
        blockerServo.setPower(0);
    }

    // Сбор шаров. Интейк сосёт. Серво блокирует путь к шутеру.
    public void collect() {
        intakeMotor.setPower(1.0);
        blockerServo.setPower(-1.0);  // крутится ОТ шутера = блокирует
    }

    // Подача к шутеру.
    public void feed() {
        intakeMotor.setPower(0.7);
        blockerServo.setPower(1.0);   // крутится К шутеру
    }

    // Остановка — не тратим энергию.
    public void stop() {
        intakeMotor.setPower(0);
        blockerServo.setPower(0);
    }

}
