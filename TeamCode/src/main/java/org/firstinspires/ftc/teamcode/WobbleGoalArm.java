package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

/*LIST OF THINGS THIS NEEDS TO BE ABLE TO DO
1. Close hand
2. Open hand
3. Move arm to "down"/tipping position
4. Move arm to straight out/placing position
5. Move arm to up/back/moving-over the wall-position
 */

//Note: Ben says there is a reduction on the arm servo
public class WobbleGoalArm {

    @Hardware
    public ServoImplEx arm;
    @Hardware
    public ServoImplEx hand;

    public WobbleGoalArm(HardwareMap hm){
        Realizer.realize(this, hm);

        //THESE NEED SET I JUST THREW THEM IN HERE
        arm.setPwmRange(new PwmControl.PwmRange(1000,2000));
        hand.setPwmRange(new PwmControl.PwmRange(1000,2000));
    }

    public void setHandPosition(double handPosition){
        hand.setPosition(handPosition);
    }

    public void setArmPosition(double armPosition){
        arm.setPosition(armPosition);
    }

    //closed hand is 1
    public void closeHand(){
        setHandPosition(1);
    }

    //open hand is 0
    public void openHand(){
        setHandPosition(0);
    }

    //this is the fully back position the arm starts in and allows us to go over the wall with the goal
    public void goToUpPosition(){
        setArmPosition(0);
    }

    //the arm sticks straight out to place the wobble goal
    public void goToPlacingPosition(){
        setArmPosition(0.5);
    }

    //this position is used to tip up a wobble goal
    public void goToDownPosition(){
        setArmPosition(1);
    }

}
