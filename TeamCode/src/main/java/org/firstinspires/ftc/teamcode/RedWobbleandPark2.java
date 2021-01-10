package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
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

        md.setAxisInversion(false, false, false);
        md.setEncoderInversion(true, true, false, false);

        f2 = new Fusion2(this, md, calculator,navSource,new PIDFCoefficients(0.8, 0, 0, 0));

        shooter = new Shooter(hardwareMap);
        wga = new WobbleGoalArm(hardwareMap);

        //the speed to which the shooter will spin up
        shooter.setSpinUpSpeed(-0.65);

        //the speed the robot will start moving at
        f2.setStartSpeed(0.2);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(300);
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(0.9);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(300);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.1);
        //this helps us not get stuck
        f2.setMinTurningSpeed(0.2);

        telemetry.log().add("Ready");
        telemetry.addData("Stackheight", height);
        telemetry.update();

        wga.closeHand();

        //waitForStart();
        // print a bit of telemetry about the pipeline while waiting for start
        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Pipeline Has Analysis", pipeline.hasAnalysis());
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
        f2.driveToPoint(new Vector2(0,-750), this::update);

        if(isStopRequested()){
            return;
        }

        //this stops the robot until the shooter comes up to speed while still running the update method
        TimingUtilities.blockUntil(this, shooter::isUpToSpeed, this::update, null);

        if(isStopRequested()){
            return;
        }

        f2.turnToHeading(Math.toRadians(0), this::update);

        if(isStopRequested()){
            return;
        }

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
                //drives to the corner zone
                f2.driveToPoint(new Vector2(0, -1600), this::update);
                if(isStopRequested()){
                    return;
                }
                wga.goToPlacingPosition();
                TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }
                wga.openHand();
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }
                //drives to get second wobble goal
                f2.driveToPoint(new Vector2(0, 2180), this::update);
                if(isStopRequested()){
                    return;
                }

                //turns to get the wobble goal
                f2.turnToHeading(Math.toRadians(90), this::update);
                if(isStopRequested()){
                    return;
                }

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

                wga.closeHand();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }

                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.5,this::update,null);
                if(isStopRequested()){
                    return;
                }

                break;

            case ONE:
                //drives to the middle zone
                f2.driveToPoint(new Vector2(0, -750), this::update);
                if(isStopRequested()){
                    return;
                }
                f2.setCrawlSpeed(0.15);

                f2.driveToPoint(new Vector2(600, -150), this::update);
                if(isStopRequested()){
                    return;
                }
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }
                wga.openHand();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }
                wga.goToUpPosition();
                TimingUtilities.sleep(this,0.6,this::update,null);
                break;

            case ZERO:
                //drives to place the first goal
                f2.driveToPoint(new Vector2(0, -400), this::update);
                if(isStopRequested()){
                    return;
                }
                //arm down
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }
                //releases hand to place first goal
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
                //lifts the arm
                wga.goToUpPosition();
                TimingUtilities.sleep(this,0.6,this::update,null);

                f2.driveToPoint(new Vector2(0,980), this::update);
                if (isStopRequested()){
                    return;
                }
                //lowers arm
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                //turns to get the wobble goal
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

                f2.setStartSpeed(0.9);
                f2.setCruiseSpeed(0.9);
                f2.setCrawlSpeed(0.9);

                f2.driveToPoint(new Vector2(-2050,-100), this::update);
                if(isStopRequested()){
                    return;
                }

                /*TimingUtilities.sleep(this,0.6,this::update, null);
                if(isStopRequested()){
                    return;
                }*/

                f2.turnToHeading(Math.toRadians(-90), this::update);
                if(isStopRequested()){
                    return;
                }

                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7, this::update, null);
                if(isStopRequested()){
                    return;
                }

                wga.openHand();
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(-300,300));
                if(isStopRequested()){
                    return;
                }

                wga.goToUpPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,0.7,this::update,null);
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