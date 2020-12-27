package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "StopOnWhite", group = "Autonomouses")
public class StopOnWhite extends LinearOpMode {
    ColorSensor beulerColorSensor;
    DcMotor backLeft, backRight, frontLeft, frontRight;


    @Override
    public void runOpMode() throws InterruptedException{

        beulerColorSensor = hardwareMap.colorSensor.get("colorSensor");
        beulerColorSensor.enableLed(true);

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(!isStopRequested()){
            telemetry.addData("Alpha:", beulerColorSensor.alpha());
            telemetry.addData("Red:", beulerColorSensor.red());
            telemetry.addData("Blue:", beulerColorSensor.blue());
            telemetry.addData("Green:", beulerColorSensor.green());
            telemetry.update();

            if (beulerColorSensor.red() < 500 && beulerColorSensor.green() < 500 && beulerColorSensor.blue() < 500){
                backLeft.setPower(0.25);
                backRight.setPower(0.25);
                frontLeft.setPower(0.25);
                frontRight.setPower(0.25);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
        }

    }
}
