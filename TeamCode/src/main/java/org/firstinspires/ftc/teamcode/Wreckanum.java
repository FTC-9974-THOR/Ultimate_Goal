package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Wreckanum", group = "Teleops")

public class Wreckanum extends OpMode{

    DcMotor rightFront, rightBack, leftFront, leftBack;
    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("MD-frontRight");
        rightBack = hardwareMap.dcMotor.get("MD-backRight");
        leftFront = hardwareMap.dcMotor.get("MD-frontLeft");
        leftBack = hardwareMap.dcMotor.get("MD-backLeft");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x;
        double rx = gamepad1.left_stick_x;

        leftFront.setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);

        telemetry.addData("Y is", + y);
        telemetry.addData("X is", + x);
    }

    @Override
    public void stop(){

    }
}

