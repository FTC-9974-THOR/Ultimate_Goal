package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "TapeMeasureExtend", group = "teleops")
public class TapeMeasureExtend extends OpMode {

    CRServo outsideServo;
    CRServo insideServo;

    @Override
    public void init() {
        outsideServo = hardwareMap.crservo.get("PT-extension0");
        insideServo = hardwareMap.crservo.get("PT-extension1");

        insideServo.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        if (gamepad1.a == true){
            outsideServo.setPower(0.5);
            insideServo.setPower(0.5);
        } else if (gamepad1.b == true) {
            outsideServo.setPower(-0.5);
            insideServo.setPower(-0.5);
        } else {
            outsideServo.setPower(0);
            insideServo.setPower(0);
        }
    }

    @Override
    public void stop(){

    }
}
