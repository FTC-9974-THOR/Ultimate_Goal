package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

@Disabled
@Autonomous(name = "Test Run To Position")
public class TestRunToPosition extends LinearOpMode {
    DcMotor backLeft, backRight, frontLeft, frontRight;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.dcMotor.get("MD-backLeft");
        backRight = hardwareMap.dcMotor.get("MD-backRight");
        frontLeft = hardwareMap.dcMotor.get("MD-frontLeft");
        frontRight = hardwareMap.dcMotor.get("MD-frontRight");

        DriveUtility driveUtility = new DriveUtility(backLeft, backRight, frontLeft, frontRight);

        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        backLeft.setMode(RUN_TO_POSITION);
        backRight.setMode(RUN_TO_POSITION);
        frontLeft.setMode(RUN_TO_POSITION);
        frontRight.setMode(RUN_TO_POSITION);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        while (backLeft.isBusy()){
            backLeft.setPower(1);
            backRight.setPower(1);
            frontLeft.setPower(1);
            frontRight.setPower(1);
        }
    }
}
