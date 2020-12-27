package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StackVisionPipeline.StackHeight;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static org.firstinspires.ftc.teamcode.StackVisionPipeline.StackHeight.FOUR;

@Autonomous(name = "RedWobbleandPark", group = "autos")
public class RedWobbleandPark extends LinearOpMode{

    DcMotor backLeft, backRight, frontLeft, frontRight;

    ColorSensor tapeDetector;

    double wheelDiameter = 96;
    double gearRatio = 27.4;
    double ticksPerRevolution = 7 * 4 * gearRatio;
    double wheelCircumference = wheelDiameter * Math.PI;
    double ticksPerMillimeter = ticksPerRevolution/wheelCircumference;

    @Override
    public void runOpMode() throws InterruptedException{
        backLeft = hardwareMap.dcMotor.get("MD-backLeft");
        backRight = hardwareMap.dcMotor.get("MD-backRight");
        frontLeft = hardwareMap.dcMotor.get("MD-frontLeft");
        frontRight = hardwareMap.dcMotor.get("MD-frontRight");

        DriveUtility driveUtility = new DriveUtility(backLeft,backRight,frontLeft,frontRight);

        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        backLeft.setMode(RUN_TO_POSITION);
        backRight.setMode(RUN_TO_POSITION);
        frontLeft.setMode(RUN_TO_POSITION);
        frontRight.setMode(RUN_TO_POSITION);

        tapeDetector = hardwareMap.colorSensor.get("colorSensor");

        StackVisionPipeline s = new StackVisionPipeline();
        StackVisionPipeline.StackHeight height = s.getAnalysis();

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (height == StackVisionPipeline.StackHeight.FOUR){
            //goes to target zone C (the one in the corner)

            int targetInMillimeters = 2820;//-2820 drove into the wall

            backLeft.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            backRight.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            frontLeft.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            frontRight.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));

            while (backLeft.isBusy()){
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(1);
            }
            /*
            //drive forward to the square
            driveUtility.driveToPosition(-2820,-1);//was -2820, 1
             */

            //backs up until it sees the tape then stops
            tapeDetector.enableLed(true);

            backLeft.setPower(0.25);
            backRight.setPower(0.25);
            frontLeft.setPower(0.25);
            frontRight.setPower(0.25);

            ElapsedTime et = new ElapsedTime();

            while(true){
                if (tapeDetector.red() > 500 && tapeDetector.green() > 500 && tapeDetector.blue() > 500){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                } else if (et.time() > 7){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                }
            }
        } else if (height == StackVisionPipeline.StackHeight.ONE){
            //goes to target zone B (the one away from the wall)

            int targetInMillimeters = -2770;

            backLeft.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            backRight.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            frontLeft.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));
            frontRight.setTargetPosition((int) (targetInMillimeters * ticksPerMillimeter));

            while (backLeft.isBusy()){
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(1);
            }

            //strafes left
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
            /*
            //drive forward to the square
           driveUtility.driveToPosition(-2770,1);
            */

            /*
            //strafe left a little bit
            driveUtility.strafeRight(560,1);
            */

            //backs up until it sees the tape and stops
            tapeDetector.enableLed(true);

            ElapsedTime et = new ElapsedTime();

            while(true){
                if (tapeDetector.red() > 500 && tapeDetector.green() > 500 && tapeDetector.blue() > 500){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                } else if (et.time() > 7){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                }
            }

        } else {
            //goes to target zone A (the one closest to the starting lines)

            /*
            driveUtility.driveToPosition(-1650,1);
            */
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

            //drive forward until it sees the tape and stops (wobble goal is considered in since it breaks the vertical plane)
            tapeDetector.enableLed(true);

            ElapsedTime et = new ElapsedTime();

            while(true){
                if (tapeDetector.red() > 500 && tapeDetector.green() > 500 && tapeDetector.blue() > 500){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                } else if (et.time() > 7){
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                }
            }
        }
    }

}
