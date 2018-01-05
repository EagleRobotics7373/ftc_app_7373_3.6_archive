package org.firstinspires.ftc.teamcode.eaglerobotics.library.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.eaglerobotics.library.functions.MathOperations;

/**
 * Created by Kk4jr on 9/28/2017.
 */

public class Holonomic extends Drivetrain {
    public Holonomic(DcMotor leftFrontMotor, DcMotor leftRearMotor, DcMotor rightFrontMotor, DcMotor rightRearMotor){
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
    }

    /**
     *
     * @param x Forward/Reverse Motion of the Robot + is Forward - is Reverse
     * @param z Rotaion of the Robot + is Clockwise - is Counter-Clockwise
     */
    public void run(double x, double z){
        run(x,0,z);
    }

    /**
     *
     * @param x Forward/Reverse Motion of the Robot + is Forward - is Reverse
     * @param y Left/Right Motion of the Robot + is Right - is Left
     * @param z Rotaion of the Robot + is Clockwise - is Counter-Clockwise
     */
    public void run(double x, double y, double z){
        x = MathOperations.rangeClip(x, -1, 1);
        y = MathOperations.rangeClip(y, -1, 1);
        z = MathOperations.rangeClip(z, -1, 1);

        double leftFrontPower = -x - y - z;
        double leftRearPower = -x + y - z;
        double rightFrontPower = x - y - z;
        double rightRearPower = x + y - z;

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    /**
     * Method to Stop the Robot
     */
    public void stop(){
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}
