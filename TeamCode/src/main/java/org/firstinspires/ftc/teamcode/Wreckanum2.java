package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

@TeleOp(name = "Wreckanum 2", group = "Teleops")
public class Wreckanum2 extends OpMode {

    DcMotor backLeft, backRight, frontLeft, frontRight;

    DriveUtility du = new DriveUtility(backLeft, backRight, frontLeft, frontRight);

    @Override
    public void init() {
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //for auto pathing use only
        backLeft.setMode(RUN_USING_ENCODER);
        backRight.setMode(RUN_USING_ENCODER);
        frontLeft.setMode(RUN_USING_ENCODER);
        frontRight.setMode(RUN_USING_ENCODER);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);*/

        double r = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        double robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;

        //calculates powers to go to each motor
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //assigns powers to the motors
        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);


        //use this for auto pathing purposes
        telemetry.addData("BackLeft: ",backLeft.getCurrentPosition()/du.ticksPerMillimeter);
        telemetry.addData("BackRight: ",backRight.getCurrentPosition()/du.ticksPerMillimeter);
        telemetry.addData("FrontLeft: ",frontLeft.getCurrentPosition()/du.ticksPerMillimeter);
        telemetry.addData("FrontRight: ",frontRight.getCurrentPosition()/du.ticksPerMillimeter);
        telemetry.update();

    }

    @Override
    public void stop(){

    }
}
