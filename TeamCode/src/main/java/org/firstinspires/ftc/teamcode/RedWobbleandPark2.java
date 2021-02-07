package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedWobbleandPark2")
public class RedWobbleandPark2 extends LinearOpMode {

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;

    private OpenCvCamera webcam;
    private Shooter shooter;
    private WobbleGoalArm wga;
    IntakeAndRamp intakeAndRamp;

    StackVisionPipeline pipeline;
    StackVisionPipeline.StackHeight height;

    ElapsedTime et;

    @Override
    public void runOpMode() throws InterruptedException {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // for some reason, streaming at 1280x720 causes EasyOpenCV to stop working. It
                // throws errors indicating it can't find a USB Streaming Endpoint. Since the FTC
                // SDK only supports streaming in uncompressed YUV420 (according to EasyOpenCV),
                // I suspect that the camera can't transfer data fast enough to stream HD YUV420.
                // if that's the case, it would explain why no endpoint was found, as the camera
                // simply doesn't have one that supports streaming in that configuration.
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //webcam.startStreaming(1280,720,OpenCvCameraRotation.UPRIGHT);
            }
        });
        //webcam.setPipeline(new StackVisionPipeline());
        // pipeline was never initialized and webcam.setPipeline() was being given a new pipeline
        // instance. pipeline stayed null and there was no way to access the pipeline that was
        // actually doing stuff.
        pipeline = new StackVisionPipeline();
        webcam.setPipeline(pipeline);

        // streaming is handled asynchronously in another thread. at this point, the pipeline may
        // not have completed processing a frame. if so, pipeline.getAnalysis() will return null.
        //height = pipeline.getAnalysis();

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(27.4, 96);
        navSource = new IMUNavSource(hardwareMap);
        intakeAndRamp = new IntakeAndRamp(hardwareMap);

        md.setAxisInversion(false, false, false);
        md.setEncoderInversion(true, true, false, false);

        f2 = new Fusion2(this, md, calculator,navSource,new PIDFCoefficients(0.8, 0, 0, 0));

        shooter = new Shooter(hardwareMap);
        wga = new WobbleGoalArm(hardwareMap);

        //the speed to which the shooter will spin up
        shooter.setSpinUpSpeed(0.7);

        //the speed the robot will start moving at
        f2.setStartSpeed(0.2);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(300);
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(1);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(300);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.1);
        //this helps us not get stuck
        f2.setMinTurningSpeed(0.2);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10,3,3,0);

        shooter.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.log().add("Ready");
        telemetry.addData("Stackheight", height);
        telemetry.update();

        wga.goToRetractedPosition();
        wga.closeHand();

        //waitForStart();
        // print a bit of telemetry about the pipeline while waiting for start
        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Pipeline Has Analysis", pipeline.hasAnalysis());
            telemetry.addData("PIDF Coefficients", shooter.flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            if (pipeline.hasAnalysis()) {
                telemetry.addData("Stack Height", pipeline.getAnalysis());
            }
            telemetry.update();
        }
        if (isStopRequested()) return;
        //we are started!

        //ramping up the shooter: method one (test against two)
        /*for (double powerIndex = 0; powerIndex <= shooter.spinUpSpeed; powerIndex += 0.01){
            TimingUtilities.sleep(this, 0.01, this:: update, null);
            shooter.setPower(powerIndex);
        }*/

        shooter.setPower(shooter.spinUpSpeed);
        if (isStopRequested()) return;
        /*
        //ramping up the shooter: method two (test against one)
        double rampSpeed = 0.5;
        ElapsedTime rampTimer = new ElapsedTime();

        while (!isStopRequested() && rampTimer.seconds() <= shooter.spinUpSpeed / rampSpeed) {
            shooter.setPower(rampSpeed * rampTimer.seconds());
        }*/

        //start the spinup right away while we're driving to the position
        //shooter.spinUp();

        // wait for the pipeline to finish processing, if necessary. unless you hit start
        // immediately after init, the pipeline should have a result by now. if it doesn't, we need
        // to wait for one.
        TimingUtilities.blockUntil(this, pipeline::hasAnalysis, this::update, null);
        if (isStopRequested()) return;
        // at this point we know that the pipeline must have a result. the only ways for the
        // blockUntil() to return is for isStopRequested() or pipeline.hasAnalysis() to return true.
        // if opmode stop was requested, the if statement would return before this line was reached.
        height = pipeline.getAnalysis();

        //drives to the position
        f2.driveToPoint(new Vector2(0,-1350), this::update);//this was 0,-750 - -350 worked pretty well too tho:: WAS -250

        if(isStopRequested()){
            return;
        }

        //this stops the robot until the shooter comes up to speed while still running the update method
        TimingUtilities.blockUntil(this, shooter::isUpToSpeed, this::update, null);

        if(isStopRequested()){
            return;
        }

        /*f2.turnToHeading(Math.toRadians(-3), this::update);//this was 0

        if(isStopRequested()){
            return;
        }*/

        //waiting for the shooter to stabilize while still running this class's update method
        TimingUtilities.sleep(this, 3, this::update, null);

        if(isStopRequested()){
            return;
        }

        //turn the robot a little bit away from the goal (-3 degrees)
        /*f2.turnToHeading(Math.toRadians(-3), this::update);

        if(isStopRequested()){
            return;
        }*/

        //tells the robot to launch the rings!
        shooter.launchRing();
        shooter.launchRing();
        shooter.launchRing();

        //waits until queuedLaunches = 0 (all the rings have been shot)
        TimingUtilities.blockUntil(this, () -> shooter.getQueuedLaunches() == 0, this::update, null);

        if(isStopRequested()){
            return;
        }

        //turns back so it is straight forward
        f2.turnToHeading(0, this::update);

        shooter.spinDown();

        switch (height){
            case FOUR:
                //4.1 drives to the corner zone
                f2.driveToPoint(new Vector2(0, -1000), this::update);
                if(isStopRequested()){
                    return;
                }
                //4.2 lowers the arm to place the first wobble goal
                wga.goToPlacingPosition();
                TimingUtilities.sleep(this,0.7,this::update, null);
                if(isStopRequested()){
                    return;
                }
                //4.3 opens the hand to let go of the first wobble goal
                wga.openHand();
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //4.4 drives to get second wobble goal
                f2.driveToPoint(new Vector2(0, 2180), this::update);//this was 0,2180
                if(isStopRequested()){
                    return;
                }

                //4.5 turns to get the wobble goal
                f2.turnToHeading(Math.toRadians(90), this::update);
                if(isStopRequested()){
                    return;
                }

                f2.setCruiseSpeed(0.7);
                f2.setStartSpeed(0.6);

                if(isStopRequested()){
                    return;
                }

                //4.6 drives to get the wobble goal
                f2.driveToPoint(new Vector2(0,-500), this::update);//this was -400
                if(isStopRequested()){
                    return;
                }
                //4.7 closes hand on the second wobble goal
                wga.closeHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //4.8 lifts arm with second wobble goal in it
                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.6,this::update,null);
                if(isStopRequested()){
                    return;
                }
                //4.9 begins drive back to the far zone
                f2.driveToPoint(new Vector2(0,500));
                if(isStopRequested()){
                    return;
                }
                //4.10 turns to original heading
                f2.turnToHeading(Math.toRadians(0));
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(1);
                f2.setCruiseSpeed(1);
                f2.setCrawlSpeed(1);
                //4.11 drives to far zone to place second wobble goal
                f2.driveToPoint(new Vector2(-100, -2300));//this was 0,-2300
                if(isStopRequested()){
                    return;
                }
                //4.12 lowers arm to place second wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7,this::update,null);
                if(isStopRequested()){
                    return;
                }
                //4.13 opens hand to let go of second wobble goal
                wga.openHand();
                if(isStopRequested()){
                    return;
                }
                //4.14 resets arm so it won't stall
                wga.setArmPosition(WobbleGoalArm.RESET_ARM_AUTO);

                TimingUtilities.sleep(this,0.8,this::update,null);
                if(isStopRequested()){
                    return;
                }
                //4.15 parks
                f2.driveToPoint(new Vector2(0, 600));
                if(isStopRequested()){
                    return;
                }

                break;

            case ONE:
                //1.1 drives forward
                f2.driveToPoint(new Vector2(0, -150), this::update);//this was 0,-750

                if(isStopRequested()){
                    return;
                }

                f2.setCrawlSpeed(0.2);
                //1.2 strafes toward center of field
                f2.driveToPoint(new Vector2(600, -150), this::update);
                if(isStopRequested()){
                    return;
                }
                //1.3 lowers arm to place first wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.8,this::update, null);
                if(isStopRequested()){
                    return;
                }
                //1.4 opens hand to place first wobble goal
                wga.openHand();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }

                /*f2.turnToHeading(Math.toRadians(180));
                if(isStopRequested()){
                    return;
                }*/
                //1.5 turns on the intake and ramp wheels to intake the ring while driving to second goal
                intakeAndRamp.setPower(1);
                //1.6 drives toward second wobble goal while intaking the ring from the starter stack
                f2.driveToPoint(new Vector2(0,900));
                if(isStopRequested()){
                    return;
                }
                //1.6 turns off ramp and intake

                //1.7 turns to grab the second wobble goal
                f2.turnToHeading(Math.toRadians(180));
                if(isStopRequested()){
                    return;
                }
                //1.8 drives to put the second wobble goal in the hand
                f2.driveToPoint(new Vector2(-100,-200));
                if(isStopRequested()){
                    return;
                }

                intakeAndRamp.setPower(0);
                //1.9 closes the hand
                wga.closeHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.8, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //1.10 raises arm
                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }
                //1.11 turns around to original head
                f2.turnToHeading(Math.toRadians(0));
                if(isStopRequested()){
                    return;
                }
                //1.12 drives across field to deliver second wobble goal
                f2.driveToPoint(new Vector2(150,-1300));
                if(isStopRequested()){
                    return;
                }
                //1.13 lowers arm to place second wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this, 0.6, this::update,null);
                //1.14 opens hand to place second wobble goal
                wga.openHand();
                if(isStopRequested()){

                }

                //the first wobble goal is now placed

                /*wga.goToUpPosition();
                TimingUtilities.sleep(this,0.6,this::update,null);*/

                /*f2.driveToPoint(new Vector2(-600, 150), this::update);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, 1300), this::update);
                if(isStopRequested()){
                    return;
                }

                f2.turnToHeading(Math.toRadians(90));
                if(isStopRequested()){
                    return;
                }

                //driving up to the wobble goal
                f2.setCruiseSpeed(0.7);
                f2.setStartSpeed(0.6);

                if(isStopRequested()){
                    return;
                }

                //drives to get the wobble goal
                f2.driveToPoint(new Vector2(0,-400), this::update);
                if(isStopRequested()){
                    return;
                }

                //closes hand on second wobble goal
                wga.closeHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //raises arm with wobble goal in hand
                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.6,this::update,null);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0,400), this::update);
                if(isStopRequested()){
                    return;
                }

                f2.turnToHeading(Math.toRadians(0));
                if(isStopRequested()){
                    return;
                }

                f2.setCruiseSpeed(1);
                f2.setStartSpeed(1);

                f2.driveToPoint(new Vector2(0, -1500));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(600, -150), this::update);
                if(isStopRequested()){
                    return;
                }

                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                wga.openHand();
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0,200));
                if(isStopRequested()){
                    return;
                }
                               */
                //1.15 raises arm
                wga.setArmPosition(WobbleGoalArm.RESET_ARM_AUTO);

                TimingUtilities.sleep(this, 0.6, this::update, null);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0,300));
                if(isStopRequested()){
                    return;
                }

                break;

            case ZERO:
                //0.2 lowers arm to place first wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }
                //0.3 opens hand to place first goal
                wga.openHand();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }

                /*f2.driveToPoint(new Vector2(300, 0), this::update);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, -400), this::update);
                if(isStopRequested()){
                    return;
                }*/
                //0.4 lifts the arm
                wga.goToUpPosition();
                TimingUtilities.sleep(this,0.6,this::update,null);

                //0.5 begins drive to get second wobble goal
                f2.driveToPoint(new Vector2(0,1080), this::update);//this was 0,980
                if (isStopRequested()){
                    return;
                }
                //0.6 lowers arm
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                //0.7 turns to get the wobble goal
                f2.turnToHeading(Math.toRadians(90), this::update);
                if(isStopRequested()){
                    return;
                }

                /*TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }*/

                //driving up to the wobble goal
                f2.setCruiseSpeed(0.7);
                f2.setStartSpeed(0.6);

                if(isStopRequested()){
                    return;
                }

                //0.8 drives to get the second wobble goal
                f2.driveToPoint(new Vector2(0,-450), this::update);//this was -400
                if(isStopRequested()){
                    return;
                }

                //0.9 closes hand on second wobble goal
                wga.closeHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //0.10 raises arm with wobble goal in hand
                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.5,this::update,null);
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(1);
                f2.setCruiseSpeed(1);
                f2.setCrawlSpeed(1);
                //0.11 drives to place the second wobble goal
                f2.driveToPoint(new Vector2(-1950,0), this::update);//this was -2050, 0
                if(isStopRequested()){
                    return;
                }

                /*TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }*/
                //0.12 turns to place second wobble goal
                f2.turnToHeading(Math.toRadians(-90), this::update);
                if(isStopRequested()){
                    return;
                }
                //0.13 lowers arm to place wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //0.14 opens hand to place second wobble goal
                wga.openHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }

                //0.15 parks
                f2.driveToPoint(new Vector2(0,300));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(-300,0));
                if(isStopRequested()){
                    return;
                }

                //0.16 resets arm so it doesn't stall
                wga.setArmPosition(WobbleGoalArm.RESET_ARM_AUTO);

                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.8,this::update,null);
                if(isStopRequested()){
                    return;
                }

                break;

        }

    }

    private void getTheGoal(){
        //turns to get the wobble goal
        f2.turnToHeading(Math.toRadians(90), this::update);
        if(isStopRequested()){
            return;
        }

        //driving up to the wobble goal
        f2.setCruiseSpeed(0.7);
        f2.setStartSpeed(0.6);

        if(isStopRequested()){
            return;
        }

        //drives to get the wobble goal
        f2.driveToPoint(new Vector2(0,-400), this::update);
        if(isStopRequested()){
            return;
        }

        //closes hand on second wobble goal
        wga.closeHand();
        if(isStopRequested()){
            return;
        }

        TimingUtilities.sleep(this,0.7, this::update, null);
        if(isStopRequested()){
            return;
        }
        //raises arm with wobble goal in hand
        wga.goToUpPosition();
        if(isStopRequested()){
            return;
        }

        TimingUtilities.sleep(this,0.5,this::update,null);
        if(isStopRequested()){
            return;
        }
    }
    private void update(){
        shooter.update();
        telemetry.addData("Flywheel speed", shooter.getFlywheelVelocity());
        telemetry.addData("Stack Height", height);
        telemetry.update();
    }
}