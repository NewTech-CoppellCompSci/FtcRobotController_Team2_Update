/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Arm Tic Testing", group="Hunter")
//@Disabled
public class Testing_ArmTic extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    /*
    Isaiah Paste This with the other instane variables
     */
    private DcMotorEx motorArm;
    private double maxTicsPerSec = 3000;
    private int targetPosition;
    private DigitalChannel armMagnet;
    private int armLevelFloor;
    private int armLevelRide;
    private int armLevelCanTilt;
    private int armLevel2nd;
    private int armLevel3rd;
    private boolean armZeroOverride;
    private String buttonPressed;

    @Override
    public void runOpMode() {


         /*
        Isaiah Paste this in init
         */
        motorArm = hardwareMap.get(DcMotorEx.class, "arm");
        motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        armMagnet = hardwareMap.get(DigitalChannel.class, "magneticSlide");
        armMagnet.setMode(DigitalChannel.Mode.INPUT);
        this.armZeroOverride = false;
        this.armLevelFloor = 10;
        this.armLevelRide = 500;
        this.armLevelCanTilt = 1340;
        this.armLevel2nd = 1015;
        this.armLevel3rd = 1650;
        this.targetPosition = 0;
        this.buttonPressed = "";


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();






        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Isaiah paste this in the main loop
             */
            //get input
            if (gamepad2.x){ //2nd level
                targetPosition = armLevel2nd;
                buttonPressed = "X";
            }
            else if (gamepad2.y){  //3rd level
                targetPosition = armLevel3rd;
                buttonPressed = "Y";
            }
            else if (gamepad2.a){  //on floor
                targetPosition = armLevelFloor;
                buttonPressed = "A";
            }
            else if (gamepad2.b || gamepad1.b){  //riding
                targetPosition = armLevelRide;
                buttonPressed = "B";
            }
            else if(gamepad2.dpad_up) {
                targetPosition += 10;
                buttonPressed = "DPad_Up";
            }
            else if (gamepad2.dpad_down) {
                targetPosition -= 10;
                buttonPressed = "Dpad_Down";
            }
            else {
                buttonPressed = "";
            }


            /*
                Keep the target position >= 0
                Unless dpad_down is pressed
             */
            if (targetPosition < 0 && !gamepad2.dpad_down){
                targetPosition = 0;
            }

            /*
                check if magnet is touching sensor
                True = NOT touching sensor
             */
            if (!armMagnet.getState()){ //not sure why it's backwards
                motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }


            //set motor
            motorArm.setTargetPosition(targetPosition);
            motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorArm.setVelocity(maxTicsPerSec);

            telemetry.addData("Arm Button: ", buttonPressed);
            telemetry.addData("At Arm Magnet: ", !armMagnet.getState());
            telemetry.addData("Can turn servo: ", motorArm.getCurrentPosition() >= armLevelCanTilt);
            telemetry.addData("Arm targetPosition: ", targetPosition);
            telemetry.addData("Arm currentPosition: ", motorArm.getCurrentPosition());
            telemetry.update();


        }
    }}