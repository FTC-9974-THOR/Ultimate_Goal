package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.concurrent.TimeUnit;

/*current velocity = acceleration * time + initial velocity
(current velocity - pastVelocity)/acceleration = time
acceleration = (currentVelocity-pastVelocity)/(currentTime-pastTime)

set currentVelocity to the setpoint (0.65 power or 3300 rpm)
solve for t to get how much time has passed when currentVelocity = setPoint

whenToPush = t (we just solved for this) minus time it takes for the pusher to push the ring

So
(0.65 - pastVelocity)/acceleration = time

WHAT DO WE WANT? time!
HOW ARE WE GONNA FIND IT? by finding pastVelocity, currentTime,and pastTime!
HOW DO WE FIND THOSE? I don't really know
HOW MUCH OF THIS CAN BE DONE WITHIN THE SHOOTER CLASS? I don't really know that either

pastTime = currentTime; //we need this at some point
pastVelocity = currentVelocity //need this somewhere too

We want to start tracking velocity and time as soon as the shooter begins spinning up
 */

//written 12/26/2020

//original PIDF coefficients: 10,3,0,0
public class Shooter {

    /*SPEED_THRESHOLD is how fast we want the shooter to be spinning when we shoot the rings
      PUSHER_MOVEMENT_TIME is the amount of time it takes for the pusher to push the ring into the flywheel
      RESET_TIME is how long it takes for the next ring to drop. If you want to change the time between shots, change this.
     */
    private static final double SPEED_THRESHOLD = 1000,//was 3200
            PUSHER_MOVEMENT_TIME= 0.35,//this was 0.5
            RESET_TIME = 1.5;

    public double spinUpSpeed;

    public double pastVelocity = 0;
    public double currentVelocity;
    public double targetVelocity;
    public double currentTime;
    public double pastTime = 0;
    public double acceleration;
    public double timeWhenAtSetpoint;
    public double howLongToPush;//need to put how long the pusher takes to push in here
    public double timeToPush = timeWhenAtSetpoint - howLongToPush;

    //public double kP,kI,kD,kF;

    //we have to add Hardware annotation to these to use the realizer
    @Hardware
    public ServoImplEx pusher;
    @Hardware
    public DcMotorEx flywheel;

    //0 to 3
    private int queuedLaunches;
    //are we in a launch cycle or not?
    private boolean inLaunchCycle;

    private ElapsedTime et;

    public Shooter(HardwareMap hm){
        Realizer.realize(this, hm);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP,kI,kD,kF));

        //we want float, not brake, because there's no reason for it to have to stop very fast
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //sets the range of the pusher (the thing that pushes the rings into the flywheel)
        pusher.setPwmRange(new PwmControl.PwmRange(1065,1250));//this was 1065, 1300

        queuedLaunches = 0;
        inLaunchCycle = false;
        et = new ElapsedTime();
    }

    public void spinUp(){
        setPower(spinUpSpeed);
    }

    public void spinDown(){
        setPower(0);
    }

    public void setPower(double power){
        flywheel.setPower(power);
    }

    //sets SpinUpSpeed, the speed at which we want to shoot the rings
    public void setSpinUpSpeed(double power){
        spinUpSpeed = power;
    }

    public double getFlywheelVelocity(){
        /*We have to do all this fancy math because REV hasn't figured out GoBilda motors yet.
          The getVelocity method returns radians per second. We divide by 2pi to get rotations per second
          We then multiply that by 60 to get rotations per minute
         */
        return flywheel.getMotorType().getGearing() * 60 * (flywheel.getVelocity(AngleUnit.RADIANS)/(2 * Math.PI));
    }

    public double getTime(){
        return et.now(TimeUnit.SECONDS);
    }

    public boolean isUpToSpeed(){

        return Math.abs(getFlywheelVelocity()) > SPEED_THRESHOLD;
    }

    public void launchRing(){
        //this constrain method makes sure that queuedLaunches doesn't go below 0 or above 3.
        queuedLaunches = (int) MathUtilities.constrain(queuedLaunches + 1, 1, 3);
    }

    //use this method to shut things down if all hell breaks loose in your program
    public void cancelLaunches(){
        queuedLaunches = 0;
        inLaunchCycle = false;
        pusher.setPosition(0);
        spinDown();
    }

    public int getQueuedLaunches() {
        return queuedLaunches;
    }

    public void setTargetVelocity(double velocity){
        targetVelocity = velocity;
    }

    public void update(){
        //checks if there are any queued launches
        if (queuedLaunches > 0){
            if(inLaunchCycle){
                //sets pusher to the "pushing" position, or 1.
                if(et.seconds() < PUSHER_MOVEMENT_TIME){
                    pusher.setPosition(1);
                    spinUp();
                    //sets pusher to the "not pushing" position, or 0.
                } else if (et.seconds() < 2 * PUSHER_MOVEMENT_TIME){
                    pusher.setPosition(0);
                    spinUp();
                    //if this is true then the next ring has dropped and we are ready to start spinning up again
                } else if (et.seconds() < 2 * PUSHER_MOVEMENT_TIME + RESET_TIME){
                    spinUp();
                } else{
                    inLaunchCycle = false;
                    queuedLaunches--;
                }
            } else if (isUpToSpeed()){
                inLaunchCycle = true;
                et.reset();
            }
        }
    }

    public void updateCalculatedShooting(){
        currentVelocity = this.getFlywheelVelocity();
        currentTime = this.getTime();

        acceleration = (currentVelocity - pastVelocity)/(currentTime - pastTime);
        timeWhenAtSetpoint = (targetVelocity-pastVelocity)/acceleration;

        timeToPush = timeWhenAtSetpoint - howLongToPush;

        if (queuedLaunches > 0){
            if (inLaunchCycle){
                if (et.seconds() >= timeToPush && et.seconds() <= 2 * timeToPush){
                    pusher.setPosition(1);
                    spinUp();
                } else if (et.seconds() >= 2 * timeToPush && et.seconds() <= 2 * timeToPush + RESET_TIME){
                    pusher.setPosition(0);
                    spinUp();
                } else if (et.seconds() >= 2 * timeToPush + RESET_TIME){
                    spinUp();
                } else{
                    inLaunchCycle = false;
                    queuedLaunches--;
                }
            } else if (isUpToSpeed()){
                inLaunchCycle = true;
                et.reset();
            }
        }
        pastTime = currentTime;
        pastVelocity = currentVelocity;

        et.reset();
    }

}