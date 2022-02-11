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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove ~or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Drive", group="Isaiah")
//@Disabled  This way it will run on the robot
public class Drive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx
    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */
    private Rev2mDistanceSensor distanceR;
    private Rev2mDistanceSensor distanceB;
    private Rev2mDistanceSensor distanceL;

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx lazyS;
    private DcMotorEx intake;
    private DcMotorEx arm;
    private Servo bucket;

    private int minHeight = 0;
    private int maxHeight = 400;
    private int currentArmPos = 0;

    static final double INCREMENT = 0.005;     // amount to slew servo each CYCLE_MS cycle
    static final double MIN_POS = 0.0;     // Maximum rotational position
    static final double MAX_POS = 1.0;     // Minimum rotational position\

    double position = 0.0; // Start at bottom position
    boolean rampUp = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        lazyS = hardwareMap.get(DcMotorEx.class, "lazyS");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        bucket = hardwareMap.get(Servo.class,"bucket");
        distanceR = hardwareMap.get(Rev2mDistanceSensor.class, "distanceR");
        distanceL = hardwareMap.get(Rev2mDistanceSensor.class, "distanceL");
        distanceB = hardwareMap.get(Rev2mDistanceSensor.class, "distanceB");





       /*
           Set up motors so they run without the encoders
           This way they run freely.  They won't go to a specific position or count the number of rotations
           It will now run with range from -1.0 to 1.0
           See Documentation for other encoder modes
           https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
        */
        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lazyS.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        position = 1.0;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //mecanum wheel code
        //https://ftccats.github.io/software/ProgrammingMecanumWheels.html

        //Get game controller input
        double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
        double rightX = (gamepad1.right_stick_x * -1);

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        double lazyPower = 0.0;
        double spinnerPower = 0.0;
        double armPower = 0.0;


        //Lazy Susan
        if (gamepad1.y || gamepad2.y) {
            lazyPower = 0.5;
            telemetry.addData("power ", "%.1f power", lazyPower);
            telemetry.update();

        }
        else if (gamepad1.x || gamepad2.x) {
            lazyPower = -0.5;
            telemetry.addData("power ", "%.1f power", lazyPower);
            telemetry.update();
        } else {
            lazyPower = 0.0;
            telemetry.addData("power ", "%.1f power", lazyPower);

        }
        //Spinner Intake
        if (gamepad1.a || gamepad2.a) {
            spinnerPower = -1.0;
            telemetry.addData("power ", "%.1f power", gamepad1.right_trigger);
            telemetry.update();

        } else {
            spinnerPower = 0.0;
            telemetry.addData("power ", "%.1f power", gamepad1.right_trigger);
        }

        //Arm
        telemetry.addData("tics ", currentArmPos);
        if ((gamepad1.right_bumper || gamepad2.right_bumper)&& currentArmPos < maxHeight) { // && currentArmPos < maxHeight
            telemetry.addData("tics ", currentArmPos);
            arm.setPower(1.0);
            currentArmPos++;

            telemetry.update();
        } else if ((gamepad1.left_bumper || gamepad2.left_bumper) && currentArmPos > minHeight) {
            //arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setPower(-1.0);
            currentArmPos--;
            telemetry.addData("tics ", currentArmPos);
            telemetry.update();
        } else {
            arm.setPower(0);
            telemetry.addData("tics ", currentArmPos);


        }

        telemetry.addData("position of servo", "%.1f", position);
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            // Keep stepping up until we hit the max value.
            //telemetry.addData("position of servo", "%.1f", position);
            position += INCREMENT ;
            bucket.setPosition(position);
            if (position >= MAX_POS) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            // Keep stepping down until we hit the min value.
            //  telemetry.addData("position of servo", "%.1f", position);
            position -= INCREMENT ;
            bucket.setPosition(position);
            if (position <= MIN_POS) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            //bucket.setPosition(position);
            telemetry.addData("position of servo", "%.1f", position);
        }


        //change the power for each wheel
        if (gamepad1.options) {
            wheelFL.setPower(v1 * 1.5);
            wheelFR.setPower(v2 * -1.5);
            wheelBL.setPower(v3 * 1.5);
            wheelBR.setPower(v4 * -1.5);
            telemetry.addData("options", "pressed");
            telemetry.update();
        } else {
            wheelFL.setPower(v1 * 0.7);
            wheelFR.setPower(v2 * -0.7);
            wheelBL.setPower(v3 * 0.7);
            wheelBR.setPower(v4 * -0.7);
            telemetry.addData("options", "not pressed");
            telemetry.update();
        }

        lazyS.setPower(lazyPower);
        intake.setPower(spinnerPower);
        arm.setPower(armPower);
        bucket.setPosition(position);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        while (currentArmPos != 0){
            arm.setPower(-1.0);
            currentArmPos--;
            telemetry.addData("tics ", currentArmPos);
            telemetry.update();
        }
    }

    public boolean distance(int cm) //cm = how far away to check
    {
        return true;
    }
}


