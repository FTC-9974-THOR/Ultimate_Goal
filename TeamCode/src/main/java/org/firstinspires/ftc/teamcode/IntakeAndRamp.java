package org.firstinspires.ftc.teamcode;


/*LIST OF THINGS THIS NEEDS TO BE ABLE TO DO:
1.Intake (intake)
2.Intake (ramp)
3.Outake (intake)
4.Outake (ramp
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class IntakeAndRamp {

    @Hardware
    public DcMotor intake;
    @Hardware
    public DcMotor ramp;
    @Hardware
    public DcMotor hopper;

    public IntakeAndRamp(HardwareMap hm){
        Realizer.realize(this,hm);

        hopper.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double power){
        intake.setPower(power);
        ramp.setPower(power);
        hopper.setPower(power);
    }
}
