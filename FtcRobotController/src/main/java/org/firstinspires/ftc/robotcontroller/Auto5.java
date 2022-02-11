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

package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file provides basic Autonomous driving for a Pushbot robot.
 * The code is structured as an Autonomous OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RedFar", group="Pushbot")
//@Disabled
public class Auto5 extends OpMode{

    /* Declare OpMode members. */
    //HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    //double          clawOffset  = 0.0 ;                  // Servo mid position
    //final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx lazyT;
    private DcMotorEx spinnerIntake;
    private DcMotorEx arm;
    //private Servo bucket;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position\

    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
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
        wheelFL  = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR  = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL  = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR  = hardwareMap.get(DcMotorEx.class, "wheelBR");
        lazyT  = hardwareMap.get(DcMotorEx.class, "lazyT");
        spinnerIntake = hardwareMap.get(DcMotorEx.class, "spinnerIntake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //bucket = hardwareMap.get(Servo.class,"bucket");



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
        spinnerIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //  arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);



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
        //travel will make the robot go in a certain direction, at a certain speed, to a certain point.
        route(4);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);
    }

    //moves in any direction, at a certain speed, for a certain amount of time
    //power = 700 for one wheel turn
    public void travel(int angle, int power, int target, int turnAngle){
        //true angle
        double trueAngle = angle - 90;

        //calculates main angle
        double r = Math.hypot(Math.cos(Math.toRadians(trueAngle)), Math.sin(Math.toRadians(trueAngle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(trueAngle)), Math.cos(Math.toRadians(trueAngle))) - Math.PI / 4;
        double rightX = 0;

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //sets target position
        wheelFL.setTargetPosition(target);
        wheelFR.setTargetPosition(target);
        wheelBL.setTargetPosition(target);
        wheelBR.setTargetPosition(target);

        // Switch to RUN_TO_POSITION mode
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //change the power for each wheel
        //this should make it turn at v1-v4 ticks per second.

        wheelFL.setVelocity(v1 * power);
        wheelFR.setVelocity(v2 * power);
        wheelBL.setVelocity(v3 * power);
        wheelBR.setVelocity(v4 * power);

//        wheelFL.setPower(v1);
//        wheelFR.setPower(v2);
//        wheelBL.setPower(v3);
//        wheelBR.setPower(v4);
    }

    //stops all
    public void stopWheels(){

        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        lazyT.setPower(0);
        spinnerIntake.setPower(0);
        arm.setPower(0);

    }

    //spins the lazy susan
    //make the power negative for reverse spin
    public void turnSusan (int power, int target){
        telemetry.addData("power ", "%.1f power", power);
        telemetry.update();

        lazyT.setTargetPosition(target);

        lazyT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lazyT.setVelocity(power);
    }

    //moves arm to a certain position at a certain speed
    //make power negative for reverse
    public void moveArm (int power, int target){
        telemetry.addData("power ", "%.1f power", power);
        telemetry.update();

        arm.setTargetPosition(target);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setVelocity(power);
    }

    //direction true = forward
    //you'll NEVER GUESS what direction set to FALSE WILL DO
    //set max to how far you want it to move forward
    //min for backwards
//    public void moveBucket (int max, int min, int increment, boolean direction){
//        if (direction) {
//            // Keep stepping up until we hit the max value.
//            position += increment ;
//            if (position >= max ) {
//                position = max;
//              //  rampUp = !rampUp;   // Switch ramp direction
//            }
//        }
//        else {
//            // Keep stepping down until we hit the min value.
//            position -= increment ;
//            if (position <= min) {
//                position = min;
//            //    rampUp = !rampUp;  // Switch ramp direction
//            }
//        }
//    }

    public void spinSpinner (int power, int rotations) {
        telemetry.addData("power ", "%.1f power", power);
        telemetry.update();

        arm.setTargetPosition(power * rotations);

        lazyT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lazyT.setVelocity(power);
    }

    //moves in any direction, at a certain speed, for a certain amount of time
    //power = 700 for one wheel turn
    public void turn(int turnAngle){

        //calculates main angle
        double r = Math.hypot(0, 0);
        double robotAngle = Math.atan2(0, 0) - Math.PI / 4;

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + turnAngle;
        final double v2 = r * Math.sin(robotAngle) - turnAngle;
        final double v3 = r * Math.sin(robotAngle) + turnAngle;
        final double v4 = r * Math.cos(robotAngle) - turnAngle;


        wheelFL.setPower(v1);
        wheelFR.setPower(v2);
        wheelBL.setPower(v3);
        wheelBR.setPower(v4);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // ignore
        }

        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);
    }
    /*
    route 1 = blue side, closest to alliance storage
    route 2 = blue side, far from alliance storage
    route 3 = red side, close
    route 4 = red side, far
    anything else = extra test routes
     */
    public void route (int route){
        if (route == 1){
            travel(45, 700, 2400, 0);
            travel(270, 700, 2400, 0);
            turn(-90);
            travel(0, 700, 9600, 0);
        }
        else if (route == 2){
            travel(45, 700, 2400, 0);
            travel(90, 700, 7200, 0);
            travel(270, 700, 2400, 0);
            turn(-90);
            travel(0, 1400, 9600, 0);
            travel(315, 700, 2400, 0);
        }
        else if (route == 3){
            travel(315, 700, 2400, 0);
            travel(90, 700, 2400, 0);
            turn(90);
            travel(0, 1400, 9600, 0);
            travel(45, 700, 2400, 0);
        }
        else if (route == 4){
            travel(315, 700, 2400, 0);
            travel(270, 700, 7200, 0);
            travel(90, 700, 2400, 0);
            turn(90);
            travel(0, 1400, 9600, 0);
            travel(45, 700, 2400, 0);
        }
        else if (route == 5){
            travel(90, 700, 2100, 0);
            travel(135, 700, 2100, 0);
            travel(180, 700, 2100, 0);
            travel(225, 700, 2100, 0);
            travel(270, 700, 2100, 0);
            travel(315, 700, 2100, 0);
            travel(0, 700, 2100, 0);
            travel(45, 700, 2100, 0);
        }
        else if (route == 6){
            turn(-90);
        }
    }
}


