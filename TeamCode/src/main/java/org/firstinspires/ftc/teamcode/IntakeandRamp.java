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

class IntakeandRamp {

    @Hardware
    public DcMotor intake;
    @Hardware
    public DcMotor ramp;

    public IntakeandRamp(HardwareMap hm){
        Realizer.realize(this,hm);
    }
}
