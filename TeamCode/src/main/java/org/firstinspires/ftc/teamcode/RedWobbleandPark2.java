package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "RedWobbleandPark2")
public class RedWobbleandPark2 extends LinearOpMode {

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;

    private Shooter shooter;
    private WobbleGoalArm wga;

    StackVisionPipeline.StackHeight height;

    @Override
    public void runOpMode() throws InterruptedException {

        StackVisionPipeline pipeline = new StackVisionPipeline();
        height = pipeline.getAnalysis();


        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(27.4, 96);
        navSource = new IMUNavSource(hardwareMap);

        md.setAxisInversion(true, true, false);
        md.setEncoderInversion(true, true, false, false);

        f2 = new Fusion2(this, md, calculator,navSource,new PIDFCoefficients(-0.8, 0, 0, 0));

        shooter = new Shooter(hardwareMap);
        wga = new WobbleGoalArm(hardwareMap);

        //the speed to which the shooter will spin up
        shooter.setSpinUpSpeed(0.65);

        //the speed the robot will start moving at
        f2.setStartSpeed(0.2);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(275);
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(0.7);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(275);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.1);
        //this helps us not get stuck
        f2.setMinTurningSpeed(0.2);

        telemetry.log().add("Ready");
        telemetry.addData("Stackheight", height);
        telemetry.update();

        waitForStart();

        //start the spinup right away while we're driving to the position
        shooter.spinUp();
        //drives to the position
        f2.driveToPoint(new Vector2(0,-850), this::update);


        if(isStopRequested()){
            return;
        }

        //this stops the robot until the shooter comes up to speed while still running the update method
        TimingUtilities.blockUntil(this, shooter::isUpToSpeed, this::update, null);

        if(isStopRequested()){
            return;
        }

        //waiting for the shooter to stabilize while still running this class's update method
        TimingUtilities.sleep(this, 4, this::update, null);

        if(isStopRequested()){
            return;
        }

        //turn the robot a little bit away from the goal (-3 degrees)
        f2.turnToHeading(Math.toRadians(-3), this::update);

        if(isStopRequested()){
            return;
        }

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

        switch (height){
            case FOUR:
                //drives to the corner zone
                f2.driveToPoint(new Vector2(0, -1000), this::update);
                wga.goToPlacingPosition();
                wga.openHand();
                break;
            case ONE:
                //drives to the middle zone
                f2.driveToPoint(new Vector2(500, -500), this::update);
                wga.goToPlacingPosition();
                wga.openHand();
                break;
            case ZERO:
                //drives to the near zone
                f2.driveToPoint(new Vector2(0, -400), this::update);
                wga.goToPlacingPosition();
                wga.openHand();
                break;

        }

        /*
        TimingUtilities.sleep(this, 1,null, null);

        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(1000, -1000));

        if(isStopRequested()){
            return;
        }

        TimingUtilities.sleep(this,1, null, null);

        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(-1000, 0));

        if(isStopRequested()){
            return;
        }
         */

    }
    private void update(){
        shooter.update();
        telemetry.addData("Flywheel speed", shooter.getFlywheelVelocity());
        telemetry.addData("Stack Height", height);
        telemetry.update();
    }
}
