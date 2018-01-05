package org.firstinspires.ftc.teamcode.eaglerobotics.library.Sensors;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Kk4jr on 11/23/2017.
 */

public class AdaIMU {
    AdafruitBNO055IMU imu;

    double[] headings = new double[3];
    double xHeading;
    double yHeading;
    double zHeading;

    double[] velocity = new double[3];
    double xVelocity;
    double yVelocity;
    double zVelocity;

    double[] acceleration = new double[3];
    double xAcceleration;
    double yAcceleration;
    double zAcceleration;

    public AdaIMU(AdafruitBNO055IMU imu){
        this.imu = imu;
    }

    public double[] getHeadings(){
        headingRefresh();
        return new double[]{xHeading, yHeading, zHeading};
    }

    private void headingRefresh(){
        Orientation temp = imu.getAngularOrientation();
        xHeading = temp.firstAngle;
        yHeading = temp.secondAngle;
        zHeading = temp.thirdAngle;
    }

    public void getVelocity(){

    }

}
