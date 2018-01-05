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

package org.firstinspires.ftc.teamcode.eaglerobotics.archive.firstmeetopmodes.firstmeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.eaglerobotics.library.drivetrain.Holonomic;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Autonomous", group = "Meet 1")
@Disabled
public class Meet1Auto extends LinearOpMode {

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

  ColorSensor colorSensorLeft;
  ColorSensor colorSensorRight;

  // Team Color and Starting Position
  Color teamColor = Color.NULL;
  //StartingPosition startingPosition = StartingPosition.NULL;

  boolean adjustment = false;

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

    colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
    colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

    // Set all servo positions here...
    jewelManipulator.setPosition(.3);

    leftIntake.setPosition(1);
    rightIntake.setPosition(0);

    // Set Team Color and Position while waiting for start
    while(!isStarted()) {
      // Set Red or Blue
      if (gamepad1.x)
        teamColor = Color.BLUE;
      else if (gamepad1.b)
        teamColor = Color.RED;

      telemetry.addData("Alliance Color: ", teamColor.toString());
      if (teamColor == Color.NULL)
        telemetry.addData("Boiiii: ", "Set the Colorrr");

      telemetry.update();
    }

    // Lower the Jewel Manipulator
    jewelManipulator.setPosition(.8);
    sleep(3000);

    // Go to each case for each color
    // Check the Color
    // Drive Forward or Backwards based on color
    String temp = "Stupid";
    switch (teamColor){
      case BLUE:
        if(colorSensorLeft.red() > colorSensorLeft.blue() && colorSensorRight.blue() > colorSensorRight.red()){
          // Drive Fwd
          holonomic.run(.5, 0, 0);
          adjustment = true;
        } else if(colorSensorLeft.red() < colorSensorLeft.blue() && colorSensorRight.blue() < colorSensorRight.red()){
          // Drive Back
          holonomic.run(-.5,0,0);
        }
        break;
      case RED:
        if(colorSensorLeft.red() < colorSensorLeft.blue() && colorSensorRight.blue() < colorSensorRight.red()){
          // Drive Fwd
          holonomic.run(.5, 0, 0);
        } else if(colorSensorLeft.red() > colorSensorLeft.blue() && colorSensorRight.blue() > colorSensorRight.red()) {
          // Drive Back
          holonomic.run(-.5, 0, 0);
          adjustment = true;
        }
        break;
      case NULL:
        telemetry.addData("YOU ARE A ", temp);
        break;
      default:
        telemetry.addData("YOU ARE A ", temp);
        break;
    }
    sleep(750);

    // Raise Arm
    jewelManipulator.setPosition(.3);
    holonomic.stop();
    sleep(500);


    /* Get Off Platform
    if(teamColor == Color.RED)
      holonomic.run(.5,0,0);
    else if(teamColor == Color.BLUE)
      holonomic.run(-.5,0,0);
    if(adjustment){
      sleep(1000);
    }
    sleep(2000);
    holonomic.stop();
    */

    // Score Block Based on Position
    /**
    switch(startingPosition){
      case LEFT:
        break;
      case RIGHT:
        break;
      case NULL:
      default:
        telemetry.addData("STUPID", "STUPID");
      break;
    }
     */

  }
}

enum Color{
  BLUE,RED,NULL
}

enum StartingPosition{
  LEFT, RIGHT, NULL
}
