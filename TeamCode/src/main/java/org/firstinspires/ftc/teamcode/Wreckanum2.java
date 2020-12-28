package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftc9974.thorcore.control.TrapezoidalMotionProfile;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

@TeleOp(name = "Wreckanum 2", group = "Teleops")
public class Wreckanum2 extends OpMode {

    DcMotor backLeft, backRight, frontLeft, frontRight;
    MecanumDrive md;

    DriveUtility du = new DriveUtility(backLeft, backRight, frontLeft, frontRight);

    IntakeAndRamp intakeAndRamp;
    WobbleGoalArm wobbleGoalArm;
    Shooter shooter;

    private boolean lastAState;
    TrapezoidalMotionProfile motionProfile;

    @Override
    public void init() {
        md = new MecanumDrive(hardwareMap);

        md.setAxisInversion(false,true, false);

        motionProfile = new TrapezoidalMotionProfile(
                new TrapezoidalMotionProfile.Node(0,0),
                new TrapezoidalMotionProfile.Node(0.5, 0.3),
                new TrapezoidalMotionProfile.Node(1,1)
        );

        intakeAndRamp = new IntakeAndRamp(hardwareMap);
        wobbleGoalArm = new WobbleGoalArm(hardwareMap);
        shooter = new Shooter(hardwareMap);

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        /*for auto pathing use only
        backLeft.setMode(RUN_USING_ENCODER);
        backRight.setMode(RUN_USING_ENCODER);
        frontLeft.setMode(RUN_USING_ENCODER);
        frontRight.setMode(RUN_USING_ENCODER);*/
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

        /*double r = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);
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
        telemetry.update();*/

        double t = Math.copySign(motionProfile.apply(Math.abs(gamepad1.left_stick_x)), -gamepad1.left_stick_x);
        md.drive(-gamepad1.right_stick_x, -gamepad1.right_stick_y,t);

        //controls the intake and the ramp
        if (gamepad1.right_trigger > 0.4 || gamepad2.right_trigger > 0.4){
            intakeAndRamp.setPower(1);
        } else if (gamepad1.left_trigger > 0.4 || gamepad2.left_trigger > 0.4){
            intakeAndRamp.setPower(-1);
        } else{
            intakeAndRamp.setPower(0);
        }

        //controls the shooter
        if (gamepad2.a){
            if (!shooter.isUpToSpeed()){
                shooter.spinUp();
            }
            if (!lastAState) {
                shooter.launchRing();
            }
        } else if (gamepad2.x){
            shooter.setSpinUpSpeed(0.65);
            shooter.spinUp();
        } else if (gamepad2.back){
            shooter.setSpinUpSpeed(0.68);
            shooter.spinUp();
        } else if (gamepad2.y){
            shooter.spinDown();
            shooter.cancelLaunches();
        }

        //see: race condition
        lastAState = gamepad2.a;

        //controls the wobble goal arm
        if (gamepad1.dpad_left){
            wobbleGoalArm.goToRetractedPosition();
        } else if (gamepad1.dpad_up){
            wobbleGoalArm.goToUpPosition();
        } else if (gamepad1.dpad_right){
            wobbleGoalArm.goToPlacingPosition();
        } else if (gamepad1.dpad_down && gamepad1.left_bumper && gamepad1.right_bumper){
            wobbleGoalArm.goToDownPosition();
        }

        if (gamepad1.right_bumper){
            wobbleGoalArm.openHand();
        } else if (gamepad1.left_bumper){
            wobbleGoalArm.closeHand();
        }

        shooter.update();
        //wobbleGoalArm.update();
    }

    @Override
    public void stop(){

    }
}
