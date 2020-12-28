package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

/*LIST OF THINGS THIS NEEDS TO BE ABLE TO DO
1. Close hand
2. Open hand
3. Move arm to "down"/tipping position
4. Move arm to straight out/placing position
5. Move arm to up/back/moving-over the wall-position
 */

//Note: Ben says there is a reduction on the arm servo
public class WobbleGoalArm {

    public static final double FULLY_RETRACTED = MathUtilities.map(650, 500, 2500, 0, 1),
                                STRAIGHT_UP = MathUtilities.map(800, 500, 2500, 0, 1),
                                GRABBING_POSITION = MathUtilities.map(2100, 500, 2500, 0, 1),
                                FULLY_DOWN = MathUtilities.map(2500, 500, 2500, 0,1),
                                CLAW_OPEN = MathUtilities.map(1000, 500, 2500, 0,1),
                                CLAW_CLOSED = MathUtilities.map(2200, 500, 2500, 0,1);

    @Hardware
    public ServoImplEx arm;
    @Hardware
    public ServoImplEx hand;

    public WobbleGoalArm(HardwareMap hm){
        Realizer.realize(this, hm);

        //THESE NEED SET I JUST THREW THEM IN HERE
        arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        hand.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    public void setHandPosition(double handPosition){
        hand.setPosition(handPosition);
    }

    public void setArmPosition(double armPosition){
        arm.setPosition(armPosition);
    }

    //closed hand is 1
    public void closeHand(){

        setHandPosition(CLAW_CLOSED);
    }

    //open hand is 0
    public void openHand() {
        setHandPosition(CLAW_OPEN);
    }

    public void goToRetractedPosition(){
        setArmPosition(FULLY_RETRACTED);
    }

    //straight up
    public void goToUpPosition() {
        setArmPosition(STRAIGHT_UP);
    }

    //the arm sticks straight out to place the wobble goal
    public void goToPlacingPosition(){
        setArmPosition(GRABBING_POSITION);
    }

    //this position is used to tip up a wobble goal
    public void goToDownPosition(){
        setArmPosition(FULLY_DOWN);
    }

}
