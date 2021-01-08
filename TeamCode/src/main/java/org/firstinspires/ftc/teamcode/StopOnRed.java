package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "StopOnRed", group = "Autonomouses")
public class StopOnRed extends LinearOpMode {

    ColorSensor beulerColorSensor;
    //DcMotor backLeft, backRight, frontLeft, frontRight;


    @Override
    public void runOpMode() throws InterruptedException {

        beulerColorSensor = hardwareMap.colorSensor.get("ASM-tapeSensor");
        beulerColorSensor.enableLed(true);

        /*backLeft = hardwareMap.dcMotor.get("MD-backLeft");
        backRight = hardwareMap.dcMotor.get("MD-backRight");
        frontLeft = hardwareMap.dcMotor.get("MD-frontLeft");
        frontRight = hardwareMap.dcMotor.get("MD-frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);*/

        waitForStart();

        while(!isStopRequested()){
            telemetry.addData("Alpha:", beulerColorSensor.alpha());
            telemetry.addData("Red:", beulerColorSensor.red());
            telemetry.addData("Blue:", beulerColorSensor.blue());
            telemetry.addData("Green:", beulerColorSensor.green());
            telemetry.update();

            /*if (beulerColorSensor.red() < 180){
                backLeft.setPower(0.25);
                backRight.setPower(0.25);
                frontLeft.setPower(0.25);
                frontRight.setPower(0.25);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }*/
        }

    }
}
