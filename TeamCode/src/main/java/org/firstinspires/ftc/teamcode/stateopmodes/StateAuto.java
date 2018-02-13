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

package org.firstinspires.ftc.teamcode.stateopmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "Autonomous LT", group = "LT")
//@Disabled
public class StateAuto extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    // Holonomic System
    DcMotor leftFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightFrontMotor;
    DcMotor rightRearMotor;

    Holonomic holonomic;

    // Lift System
    // Threaded rod lift
    DcMotor lift;
    DcMotor intake;

    // Jewel Manipulator
    Servo jewelManipulator;
    Servo jewelRotator;

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;

    BNO055IMU imu;

    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark visibleVuMark = RelicRecoveryVuMark.UNKNOWN;

    // Team Color and Starting Position
    Color teamColor = Color.NULL;
    StartingPosition startingPosition = StartingPosition.NULL;

    GlobalVarsState vars = new GlobalVarsState();

    // Arrays to store angle values for each position
    // L , C , R
    int[] RedLeft = {-105 , -77, -50};
    int[] RedRight = {122, 133, 167};
    int[] BlueLeft = {0, 20, 0};
    int[] BlueRight = {-100, -85, -65};

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Get motors from map
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        holonomic = new Holonomic(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");

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
        jewelManipulator.setPosition(vars.jewelManipulatorTopPosition);
        jewelRotator.setPosition(vars.jewelRotatorMidPosition);

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

        // Get vuMark
        // Make 3 attempts every 500ms

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        for(int i = 0; i < 3; i++){
            if( vuMark != RelicRecoveryVuMark.UNKNOWN)
                break;
            sleep(500);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        // Force Center
        // vuMark = RelicRecoveryVuMark.UNKNOWN;

        telemetry.addData("Vumark: ", vuMark.toString());
        telemetry.update();

        // Lower Jewel Manipulator
        jewelManipulator.setPosition(vars.jewelManipulatorMiddlePosition);
        sleep(1000);
        jewelManipulator.setPosition(vars.jewelManipulatorLoweredPosition);
        sleep(3000);

        switch(teamColor){
            case BLUE:
                telemetry.addData("Color :", "Blue");
                telemetry.update();
                // Check color condition and move jewelRotator
                /*
                if(colorSensorLeft.blue() > colorSensorRight.blue() &&
                        colorSensorRight.red() > colorSensorLeft.red()){
                    jewelRotator.setPosition(vars.jewelRotatorRightPosition);
                    telemetry.addData("Right", "Blah");
                    telemetry.update();
                } else if(colorSensorLeft.blue() < colorSensorRight.blue() &&
                        colorSensorRight.red() < colorSensorLeft.red()){
                    jewelRotator.setPosition(vars.jewelRotatorLeftPosition);
                    telemetry.addData("Left", "Blah");
                    telemetry.update();
                } else {

                }
                */

                // Check color condition and move jewelRotator
                if(colorSensorRight.red() > colorSensorRight.blue()){
                    jewelRotator.setPosition(vars.jewelRotatorRightPosition);
                    telemetry.addData("Right", "Blah");
                    telemetry.update();
                } else if(colorSensorLeft.red() > colorSensorLeft.blue()){
                    jewelRotator.setPosition(vars.jewelRotatorLeftPosition);
                    telemetry.addData("Left", "Blah");
                    telemetry.update();
                } else {

                }

                // Set position for jewel arm and grab block/raise lift
                sleep(1000);
                resetLiftandJewel();


                // Drive Forward
                holonomic.run(.5, 0, 0);


                switch(startingPosition){
                    case LEFT:
                        // How long to drive forward
                        sleep(1000);

                        // Rotate based on vuMark
                        switch (vuMark){
                            case LEFT:
                                telemetry.addData("VuMark :", "Left");
                                telemetry.update();
                                rotateWide(BlueLeft[0]);
                                break;
                            case UNKNOWN:
                                telemetry.addData("VuMark : !!!", "Null!!!");
                            case CENTER:
                                telemetry.addData("VuMark :", "Center");
                                telemetry.update();
                                rotate(BlueLeft[1]);
                                break;
                            case RIGHT:
                                telemetry.addData("VuMark :", "Right");
                                telemetry.update();
                                rotate(BlueLeft[2]);
                                break;
                        }

                        break;
                    case RIGHT:
                        // How long to drive forward
                        sleep(1300);


                        // Rotate based on vuMark
                        switch (vuMark){
                            case LEFT:
                                telemetry.addData("VuMark :", "Left");
                                telemetry.update();
                                rotateWide(BlueRight[0]);
                                break;
                            case UNKNOWN:
                                telemetry.addData("VuMark : !!!", "Null!!!");
                            case CENTER:
                                telemetry.addData("VuMark :", "Center");
                                telemetry.update();
                                rotate(BlueRight[1]);
                                break;
                            case RIGHT:
                                telemetry.addData("VuMark :", "Right");
                                telemetry.update();
                                rotate(BlueRight[2]);
                                break;
                        }
                        break;
                }
                break;
            case RED:
                telemetry.addData("Color :", "Red");
                telemetry.update();
                // Check color condition and move jewelRotator
                /*
                if(colorSensorLeft.blue() < colorSensorRight.blue() &&
                        colorSensorRight.red() < colorSensorLeft.red()){
                    jewelRotator.setPosition(vars.jewelRotatorRightPosition);
                } else if(colorSensorLeft.blue() > colorSensorRight.blue() &&
                        colorSensorRight.red() > colorSensorLeft.red()){
                    jewelRotator.setPosition(vars.jewelRotatorLeftPosition);
                }*/
                if(colorSensorRight.blue() > colorSensorRight.red()){
                    jewelRotator.setPosition(vars.jewelRotatorRightPosition);
                    telemetry.addData("Right", "Blah");
                    telemetry.update();
                } else if(colorSensorLeft.blue() > colorSensorLeft.red()){
                    jewelRotator.setPosition(vars.jewelRotatorLeftPosition);
                    telemetry.addData("Left", "Blah");
                    telemetry.update();
                } else {
                }

                sleep(1000);
                resetLiftandJewel();

                // Drive backward
                holonomic.run(-.5, 0, 0);

                switch(startingPosition){
                    case LEFT:
                        // How long to drive backward
                        sleep(1750);


                        // Rotate based on vuMark
                        switch (vuMark){
                            case LEFT:
                                telemetry.addData("VuMark :", "Left");
                                telemetry.update();
                                rotateWide(RedLeft[0]);
                                break;
                            case UNKNOWN:
                                telemetry.addData("VuMark : !!!", "Null!!!");
                            case CENTER:
                                telemetry.addData("VuMark :", "Center");
                                telemetry.update();
                                rotate(RedLeft[1]);
                                break;
                            case RIGHT:
                                telemetry.addData("VuMark :", "Right");
                                telemetry.update();
                                rotate(RedLeft[2]);
                                break;
                        }
                        break;
                    case RIGHT:
                        // How long to drive backward
                        sleep(1000);

                        // Rotate based on vuMark
                        switch (vuMark){
                            case LEFT:
                                telemetry.addData("VuMark :", "Left");
                                telemetry.update();
                                rotateWide(RedRight[0]);
                                break;
                            case UNKNOWN:
                                telemetry.addData("VuMark : !!!", "Null!!!");
                            case CENTER:
                                telemetry.addData("VuMark :", "Center");
                                telemetry.update();
                                rotateWide(RedRight[1]);
                                break;
                            case RIGHT:
                                telemetry.addData("VuMark :", "Right");
                                telemetry.update();
                                rotateWide(RedRight[2]);
                                break;
                        }
                        break;
                }
                break;
        }

        // Drive forward into scoring zone

        holonomic.run(.5,0, 0);
        sleep(1250);
        holonomic.stop();
        sleep(1000);

        lift.setPower(.5);
        sleep(250);
        lift.setPower(0);
        sleep(500);

        holonomic.run(.5,0, 0);
        sleep(500);
        holonomic.stop();
        sleep(1000);

        // Open Intake
        intake.setPower(1);
        sleep(750);
        intake.setPower(0);
        sleep(1000);

        // Back away but stay in the safe zone
        holonomic.run(-.3, 0, 0);
        sleep(500);
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


    private void rotateOldFashioned(int target){
        target = -target;
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

    void rotateWide(int target){
        target = -target;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        time.startTime();
        float temp = imuXAngle();
        while(time.seconds() <  7 && opModeIsActive()) {
            temp = imuXAngle();
            telemetry.addData("IMU: ", temp);
            telemetry.update();
            if (temp > target) {
                holonomic.run(0, 0, .35 - (time.seconds()*.05) );
            } else if (temp < target) {
                holonomic.run(0, 0, -.35 + (time.seconds()*.05) );
            }
        }
        holonomic.stop();
    }

    void rotate(int target){
        target = -target;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        time.startTime();
        float temp = imuXAngle();
        while(time.seconds() <  5 && opModeIsActive()) {
            temp = imuXAngle();
            telemetry.addData("IMU: ", temp);
            telemetry.update();
            if (temp > target) {
                    holonomic.run(0, 0, .25 - (time.seconds()*.05) );
            } else if (temp < target) {
                    holonomic.run(0, 0, -.25 + (time.seconds()*.05) );
            }
        }
        holonomic.stop();
    }

    private void resetLiftandJewel(){
        // Set position for jewel arm and grab block/raise lift
        intake.setPower(-1);
        jewelManipulator.setPosition(vars.jewelManipulatorMiddlePosition);
        sleep(2000);
        lift.setPower(-1);
        jewelRotator.setPosition(vars.jewelRotatorStoredPosition);
        jewelManipulator.setPosition(vars.jewelManipulatorStoredPosition);
        sleep(250);
        lift.setPower(0);
    }

}

enum Color {
    BLUE, RED, NULL
}

enum StartingPosition {
    LEFT, RIGHT, NULL
}

