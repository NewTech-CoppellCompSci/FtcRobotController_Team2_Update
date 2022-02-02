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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="Run To Position Testing", group="Hunter")
//@Disabled
public class RunToPositionTesting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    /*
      Declare motors to type DcMotorEx

      Documentation:
      https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
   */
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;

    /*
        max speed for the drive motors.
        I think max is 600
     */
    private double maxDriveTicsPerSec = 4000;

    private int loopCount = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFL  = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR  = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL  = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR  = hardwareMap.get(DcMotorEx.class, "wheelBR");


        /*
            Set up motors so they run without the encoders
            This way they run freely.  They won't go to a specific position or count the number of rotations
            It will now run with range from -1.0 to 1.0

            See Documentation for other encoder modes
            https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
         */
        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);




        // In RUN_USING_ENCODER mode, they all need to run in reverse
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        //set PIDFCoefficients
        wheelFL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelFR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //pause program till start is pressed on Driver Hub
        waitForStart();


        basicEncoderTesting();


    }


    /*
        runs each wheel forward then backwards for 3 seconds
        order: FL, BL, FR, BR
     */
    public void basicEncoderTesting(){

        //wheelFL
        wheelFL.setTargetPosition(4000);
        wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFL.setVelocity(maxDriveTicsPerSec / 10);
        while (opModeIsActive()) {
            telemetry.addData("velocity", wheelFL.getVelocity());
            telemetry.addData("position", wheelFR.getCurrentPosition());
            telemetry.addData("is at target", !wheelFL.isBusy());
            telemetry.update();
        }
        while(wheelFL.isBusy()) {}



        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFL.setTargetPosition(-4000);
        wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFL.setVelocity(maxDriveTicsPerSec / 2);
        while (opModeIsActive()) {
            telemetry.addData("velocity", wheelFL.getVelocity());
            telemetry.addData("position", wheelFR.getCurrentPosition());
            telemetry.addData("is at target", !wheelFL.isBusy());
            telemetry.update();
        }
    }

}
