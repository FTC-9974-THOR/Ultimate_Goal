package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "RedOneGoalShootAndPark", group = "autnomouses")
public class RedOneGoalShootAndPark extends LinearOpMode {

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;


    private OpenCvCamera webcam;
    private Shooter shooter;
    private WobbleGoalArm wga;
    IntakeAndRamp intakeAndRamp;
    Blocker blocker;

    StackVisionPipeline pipeline;
    StackVisionPipeline.StackHeight height;

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

        pipeline = new StackVisionPipeline();
        webcam.setPipeline(pipeline);

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(27.4, 96);
        navSource = new IMUNavSource(hardwareMap);
        intakeAndRamp = new IntakeAndRamp(hardwareMap);
        blocker = new Blocker(hardwareMap);


        md.setAxisInversion(false, false, false);
        md.setEncoderInversion(true, true, false, false);

        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0));

        shooter = new Shooter(hardwareMap);
        wga = new WobbleGoalArm(hardwareMap);

        shooter.setSpinUpSpeed(0.66);//was 0.7

        //the speed the robot will start moving at
        f2.setStartSpeed(0.1);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(150);
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(1);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(150);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.1);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10,3,3,0);

        shooter.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.log().add("Ready");
        telemetry.addData("Stackheight", height);
        telemetry.update();

        wga.goToRetractedPosition();
        wga.closeHand();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Pipeline Has Analysis", pipeline.hasAnalysis());
            telemetry.addData("PIDF Coefficients", shooter.flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            if (pipeline.hasAnalysis()) {
                telemetry.addData("Stack Height", pipeline.getAnalysis());
            }
            telemetry.update();
        }
        if (isStopRequested()){
            return;
        }

        shooter.setPower(shooter.spinUpSpeed);
        if(isStopRequested()){
            return;
        }

        TimingUtilities.blockUntil(this, pipeline::hasAnalysis, this::update, null);
        if(isStopRequested()){
            return;
        }

        height = pipeline.getAnalysis();

        f2.driveToPoint(new Vector2(0,-1350), this::update);//this was 0,-1350

        if(isStopRequested()){
            return;
        }

        TimingUtilities.blockUntil(this, shooter::isUpToSpeed, this::update, null);
        if(isStopRequested()){
            return;
        }

        TimingUtilities.sleep(this, 1, this::update, null);
        if(isStopRequested()){
            return;
        }

        f2.turnToHeading(Math.toRadians(6), this::update);
        if(isStopRequested()){
            return;
        }

        shooter.launchRing();
        shooter.launchRing();
        shooter.launchRing();

        TimingUtilities.blockUntil(this, () -> shooter.getQueuedLaunches() == 0, this::update, null);
        if(isStopRequested()){
            return;
        }

        f2.turnToHeading(0, this::update);
        if(isStopRequested()){
            return;
        }

        shooter.spinDown();

        switch(height){
            case FOUR:
                f2.driveToPoint(new Vector2(0, -1000), this::update);//this was 0,-1000
                if(isStopRequested()){
                    return;
                }

                wga.goToPlacingPosition();
                TimingUtilities.sleep(this,0.8,this::update, null);
                if(isStopRequested()){
                    return;
                }

                wga.openHand();
                TimingUtilities.sleep(this,0.5, this::update, null);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, 750), this::update);//this was 0,2180
                if(isStopRequested()){
                    return;
                }

                wga.goToRetractedPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this, 1, this::update, null);
                if(isStopRequested()){
                    return;
                }

                break;
            case ONE:
                f2.driveToPoint(new Vector2(0, -150), this::update);//this was 0,-750
                if(isStopRequested()){
                    return;
                }

                f2.setCrawlSpeed(0.2);
                //1.2 strafes toward center of field
                f2.driveToPoint(new Vector2(600, -350), this::update);//this was 600,-150
                if(isStopRequested()){
                    return;
                }

                //1.3 lowers arm to place first wobble goal
                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this,1,this::update, null);//this was 0.8
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

                f2.driveToPoint(new Vector2(0,400));//this was 0,1100
                if(isStopRequested()){
                    return;
                }

                wga.goToRetractedPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this, 1, this::update, null);
                if(isStopRequested()){
                    return;
                }

                break;
            case ZERO:

                wga.goToPlacingPosition();
                if(isStopRequested()){
                    return;
                }
                TimingUtilities.sleep(this,1,this::update, null);
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

                wga.goToUpPosition();
                TimingUtilities.sleep(this,0.6,this::update,null);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(600,0), this::update);//this was 0,980
                if (isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0,-150));
                if(isStopRequested()){
                    return;
                }

                wga.goToRetractedPosition();
                if(isStopRequested()){
                    return;
                }

                TimingUtilities.sleep(this, 1, this::update, null);
                if(isStopRequested()){
                    return;
                }
        }
    }

    private void update(){
        shooter.update();
        telemetry.addData("Flywheel speed", shooter.getFlywheelVelocity());
        telemetry.addData("Stack Height", height);
        telemetry.update();
    }
}
