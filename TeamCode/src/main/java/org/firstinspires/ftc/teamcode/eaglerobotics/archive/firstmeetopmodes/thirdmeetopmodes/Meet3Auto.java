/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.eaglerobotics.archive.firstmeetopmodes.thirdmeetopmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.eaglerobotics.library.drivetrain.Holonomic;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Autonomous Meet 3", group = "Meet 3")
@Disabled
public class Meet3Auto extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    // Holonomic System
    DcMotor leftFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightFrontMotor;
    DcMotor rightRearMotor;

    Holonomic holonomic;

    // Lift System
    // Threaded rod lift
    DcMotor leftThreadedRodLift;
    DcMotor rightThreadedRodLift;

    // Intake/Scorer System
    // Intake
    Servo leftIntake;
    Servo rightIntake;

    // Jewel Manipulator
    Servo jewelManipulator;
    Servo jewelRotator;

    double leftPosition = 0;
    double rightPosition = 1;
    double middlePosition = .5;

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;

    BNO055IMU imu;

    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark visibleVuMark = RelicRecoveryVuMark.UNKNOWN;

    // Team Color and Starting Position
    Color teamColor = Color.NULL;
    StartingPosition startingPosition = StartingPosition.NULL;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Get motors from map
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        holonomic = new Holonomic(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        leftThreadedRodLift = hardwareMap.dcMotor.get("leftThreadedRodLift");
        rightThreadedRodLift = hardwareMap.dcMotor.get("rightThreadedRodLift");

        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");

        jewelManipulator = hardwareMap.servo.get("jewelManipulator");
        jewelRotator = hardwareMap.servo.get("jewelRotator");

        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set all servo positions here...
        jewelManipulator.setPosition(1);
        jewelRotator.setPosition(middlePosition);

        leftIntake.setPosition(1);
        rightIntake.setPosition(0);

        // Set up Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersVu.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parametersVu.vuforiaLicenseKey = "AcMSLB//////AAAAGV2X9BmFFk6Pt9dw+Dg7oCSDbgmpvFL2uaQFUQNenTRFP8eywDy/1JH+6MeeMp/aHH3L2pWVW+t2hx9saq2n72eE+/6orS0hL6ooUobxBlvKS6YQqJIQM7ZOTOIVVpgpzVODNQVdcvRW6Vm2yGrRUAPnuEScnQU9ahY8PSApozJ05M8oS33fEP8T76Y8V31jWRqaw1JIsXQRKHzmQpK5l1no4LwBQ/iCxmHHJ3h77zlfKDsP9DQrh0r/r9b8dP7sSMtCQsukfrmwD4o5uF+S6e4ScWTA4tgpXkPMYVfyjVLsynvNHhi2kuzd2goDeP1uNgpSoEXzJQQKcNeo99nKm3BU22USUBPliFrocMRYGnxb";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVu);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        // Set Team Color and Position while waiting for start
        while (!isStarted()) {
            // Set Red or Blue
            if (gamepad1.x)
                teamColor = Color.BLUE;
            else if (gamepad1.b)
                teamColor = Color.RED;

             if (gamepad1.right_bumper) {
             startingPosition = StartingPosition.RIGHT;
             } else if (gamepad1.left_bumper) {
             startingPosition = StartingPosition.LEFT;
             }


            telemetry.addData("Alliance Color: ", teamColor.toString());
            telemetry.addData("Starting Position: ", startingPosition.toString());

            telemetry.update();
        }

        // Lower the Jewel Manipulator
        jewelManipulator.setPosition(0);
        sleep(3000);

        // Go to each case for each color
        // Check the Color
        // Rotate Forward or Backwards based on color
        if(teamColor == Color.BLUE){
            if(colorSensorLeft.blue() > colorSensorLeft.red() && colorSensorRight.red() > colorSensorRight.blue()){
                jewelRotator.setPosition(rightPosition);
                telemetry.addData("B", "R");

            } else if(colorSensorRight.blue() > colorSensorRight.red() && colorSensorLeft.red() > colorSensorLeft.blue()){
                jewelRotator.setPosition(leftPosition);
                telemetry.addData("B", "L");
            }

        } else if (teamColor == Color.RED) {
            if(colorSensorLeft.blue() > colorSensorLeft.red() && colorSensorRight.red() > colorSensorRight.blue()){
                jewelRotator.setPosition(leftPosition);
                telemetry.addData("R", "L");
            } else if(colorSensorRight.blue() > colorSensorRight.red() && colorSensorLeft.red() > colorSensorLeft.blue()){
                jewelRotator.setPosition(rightPosition);
                telemetry.addData("R", "R");
            }
        }
        telemetry.update();

        sleep(500);
        jewelManipulator.setPosition(.5);
        sleep(500);
        jewelRotator.setPosition(middlePosition);
        sleep(500);
        jewelManipulator.setPosition(1);

        // Grab that block
        leftIntake.setPosition(.5);
        rightIntake.setPosition(.5);

        // Rasie the lift just barely
        leftThreadedRodLift.setPower(1);
        rightThreadedRodLift.setPower(1);
        sleep(500);
        leftThreadedRodLift.setPower(0);
        rightThreadedRodLift.setPower(0);
        sleep(500);

        // Drive Off Platform
        if(teamColor == Color.BLUE){
            holonomic.run(-.5,0,0);
            if(startingPosition == StartingPosition.LEFT){
                sleep(1600);
            }else if(startingPosition == StartingPosition.RIGHT){
                sleep(2500);
            }
        } else if(teamColor == Color.RED){
            holonomic.run(.5,0,0);
            if(startingPosition == StartingPosition.LEFT){
                sleep(1800);
            }else if(startingPosition == StartingPosition.RIGHT){
                sleep(1000);
            }
        }

        // Rotate to line up to score
        if(teamColor == Color.BLUE){
            if(startingPosition == StartingPosition.LEFT){
               rotate(130);
            }else if(startingPosition == StartingPosition.RIGHT){
                rotate(-55);
            }
        } else if(teamColor == Color.RED){
            if(startingPosition == StartingPosition.LEFT){
                rotate(-55);
            }else if(startingPosition == StartingPosition.RIGHT){
                rotate(20);
            }
        }
        holonomic.stop();
        // Drive forward to score
        holonomic.run(.5,0,0);
        sleep(1500);
        holonomic.stop();

        //Release and Backup
        leftIntake.setPosition(1);
        rightIntake.setPosition(0);
        sleep(1000);
        holonomic.run(-.5, 0, 0);
        sleep(250);
        holonomic.stop();



    }

    private Orientation orientation(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private float[] imuAngles(){
           Orientation angles = imu.getAngularOrientation();
           return new float[]{angles.firstAngle, angles.secondAngle, angles.thirdAngle};
    }

    private float imuXAngle(){
        return imuAngles()[0];
    }

    private float imuYAngle(){
        return imuAngles()[1];
    }
    private float imuZAngle(){
        return imuAngles()[2];
    }

    private void rotate(int target){
        if (target < 0) {
            float temp = imuXAngle();
            while (temp > target && opModeIsActive()) {
                holonomic.run(0, 0, .2);

                telemetry.addData("IMU: ", temp);
                telemetry.update();

                temp = imuXAngle();
            }
        } else if (target > 0) {
            float temp = imuXAngle();
            while (temp < target && opModeIsActive()) {
                holonomic.run(0, 0, -.2);

                telemetry.addData("IMU: ", temp);
                telemetry.update();

                temp = imuXAngle();
            }
        }
        holonomic.stop();
    }
}

enum Color {
    BLUE, RED, NULL
}

enum StartingPosition {
    LEFT, RIGHT, NULL
}
