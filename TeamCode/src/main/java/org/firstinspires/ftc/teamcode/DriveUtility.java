package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveUtility extends LinearOpMode{

    public DcMotor fl, fr, bl, br;

    public DriveUtility(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight){
             bl = backLeft;
             br = backRight;
             fl = frontLeft;
             fr = frontRight;
    }

    double wheelDiameter = 96;
    double gearRatio = 27.4;
    double ticksPerRevolution = 7 * 4 * gearRatio;
    double wheelCircumference = wheelDiameter * Math.PI;
    double ticksPerMillimeter = ticksPerRevolution/wheelCircumference;


    public void turnToAngle(int turnAngle, double power) {

        if (isStopRequested()) {
            return;
        }

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) {
            return;
        }

        int target = (int)(turnAngle * wheelDiameter/2);

        if (turnAngle > 0) {

            fl.setTargetPosition(target);
            fr.setTargetPosition(target);
            bl.setTargetPosition(target);
            br.setTargetPosition(target);

            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        }
        else if (turnAngle < 0) {
            fl.setTargetPosition(target);
            fr.setTargetPosition(target);
            bl.setTargetPosition(target);
            br.setTargetPosition(target);

            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
        }

        if (isStopRequested()) {
            return;
        }
    }



    /**private void turnLeft(int turnAngle, double power) {

     if (isStopRequested()) {
     return;
     }

     fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

     fl.setDirection(DcMotorSimple.Direction.REVERSE);
     bl.setDirection(DcMotorSimple.Direction.REVERSE);

     if (isStopRequested()) {
     return;
     }

     double wheelRadius = 2;
     int target = (int)(turnAngle * wheelRadius);

     fl.setTargetPosition(target);
     fr.setTargetPosition(target);
     bl.setTargetPosition(target);
     br.setTargetPosition(target);

     fl.setPower(-power);
     fr.setPower(power);
     bl.setPower(-power);
     br.setPower(power);

     if (isStopRequested()) {
     return;
     }
     }**/



    public void driveToPosition(double distanceInMilimeters, double power) {

        if (isStopRequested()) {
            return;
        }

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) {
            return;
        }

        int target = (int) (distanceInMilimeters * ticksPerRevolution);

        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        if (isStopRequested()) {
            return;
        }
    }



    public void strafeRight(double distanceInMilimeters, double power) {

        if (isStopRequested()) {
            return;
        }

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) {
            return;
        }

        int target = (int) (distanceInMilimeters * ticksPerRevolution);

        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);

        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);

        if (isStopRequested()) {
            return;
        }
    }

    public void strafeLeft(double distanceInMilimeters, double power) {

        if (isStopRequested()) {
            return;
        }

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) {
            return;
        }

        int target = (int) (distanceInMilimeters * ticksPerRevolution);

        fl.setTargetPosition(-target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(-target);

        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);

        if (isStopRequested()) {
            return;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
