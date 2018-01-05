package org.firstinspires.ftc.teamcode.eaglerobotics.library.encoder;

import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Kk4jr on 11/4/2017.
 */

public class EncoderMotor {
    DcMotor motor;
    float powerMax;

    public EncoderMotor(DcMotor motor, float powerMax){
        this.motor = motor;
        this.powerMax = powerMax;
    }

    public void runToPosition(int position){
        powerSetBasedOnPosition(position);
    }

    private void powerSetBasedOnPosition(int position){
        if(motor.getCurrentPosition() > position + 500){
            motor.setPower(-powerMax);
        }else if(motor.getCurrentPosition() < position - 500){
            motor.setPower(powerMax);
        }if(motor.getCurrentPosition() > position + 100){
            motor.setPower(-.5*powerMax);
        }else if(motor.getCurrentPosition() < position - 100){
            motor.setPower(.5*powerMax);
        } else {
            motor.setPower(0);
        }
    }

}
