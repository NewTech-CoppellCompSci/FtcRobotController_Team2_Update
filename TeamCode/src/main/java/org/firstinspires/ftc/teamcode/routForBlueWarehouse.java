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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="Main Blue Auto", group="Linear Opmode")
//@Disabled
public class routForBlueWarehouse extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    //double          clawOffset  = 0.0 ;                  // Servo mid position
    //final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx lazyS;
    private DcMotorEx intake;
    private DcMotorEx arm;

    private Rev2mDistanceSensor distanceR;
    private Rev2mDistanceSensor distanceB;
    private Rev2mDistanceSensor distanceL;

    private DcMotorEx motorArm;
    private double maxTicsPerSec = 3000;
    private int targetPosition = 0;
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

        SETUP!!

         */
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFL  = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR  = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL  = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR  = hardwareMap.get(DcMotorEx.class, "wheelBR");
        lazyS = hardwareMap.get(DcMotorEx.class, "lazyS");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        distanceR = hardwareMap.get(Rev2mDistanceSensor.class, "distanceR");
        distanceL = hardwareMap.get(Rev2mDistanceSensor.class, "distanceL");
        distanceB = hardwareMap.get(Rev2mDistanceSensor.class, "distanceB");

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
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lazyS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        //pause program till start is pressed on Driver Hub
        waitForStart();

        route(1);





    }

    //moves in any direction, at a certain speed, for a certain amount of time
    //moves straight
    //power = 700 for one wheel turn
    public void travel(int angle, int power, int target){
        wheelFL.setTargetPositionTolerance(10);
        wheelFR.setTargetPositionTolerance(10);
        wheelBL.setTargetPositionTolerance(10);
        wheelBR.setTargetPositionTolerance(10);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //true angle
        double trueAngle = (angle - 90);

        //number of 45 degree angles from 0 (unit Circle) (in radians)
        double num45s = Math.toRadians(trueAngle / 45);
//        if (angle == 0){
//            num45s = 7;
//        }


        //calculates main angle
        double r = Math.hypot(Math.cos(Math.toRadians(trueAngle)), Math.sin(Math.toRadians(trueAngle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(trueAngle)), Math.cos(Math.toRadians(trueAngle))) - ((Math.PI / 4) + ((Math.PI / 4) * num45s));

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = r * Math.cos(robotAngle);

        //sets target position
        wheelFL.setTargetPosition(v1 > 0 ? target : -1 * target); //v1 > 0 ? target : -1 * target
        wheelFR.setTargetPosition(v2 > 0 ? target : -1 * target);
        wheelBL.setTargetPosition(v3 > 0 ? target : -1 * target);
        wheelBR.setTargetPosition(v4 > 0 ? target : -1 * target);

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

        telemetry.addData("power ", "%.1f power", v1);
        telemetry.addData("power ", "%.1f power", v2);
        telemetry.addData("power ", "%.1f power", v3);
        telemetry.addData("power ", "%.1f power", v4);
        telemetry.update();

        while(wheelFL.isBusy()  ||  wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("FL Status", wheelFL.isBusy());
            telemetry.addData("FR Status", wheelFR.isBusy());
            telemetry.addData("BL Status", wheelBL.isBusy());
            telemetry.addData("BR Status", wheelBR.isBusy());
            telemetry.update();


        }
//        wheelFL.setPower(v1);
//        wheelFR.setPower(v2);
//        wheelBL.setPower(v3);
//        wheelBR.setPower(v4);
    }

    //moves in any direction, at a certain speed, for a certain amount of time
    //moves straight
    //power = 700 for one wheel turn
    public void turn(int angle, int power, int target){
        wheelFL.setTargetPositionTolerance(5);
        wheelFR.setTargetPositionTolerance(5);
        wheelBL.setTargetPositionTolerance(5);
        wheelBR.setTargetPositionTolerance(5);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.FORWARD);

        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //true angle
        double trueAngle = (angle - 90);

        //number of 45 degree angles from 0 (unit Circle) (in radians)
        double num45s = Math.toRadians(trueAngle / 45);
//        if (angle == 0){
//            num45s = 7;
//        }


        //calculates main angle
        double r = Math.hypot(Math.cos(Math.toRadians(trueAngle)), Math.sin(Math.toRadians(trueAngle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(trueAngle)), Math.cos(Math.toRadians(trueAngle))) - ((Math.PI / 4) + ((Math.PI / 4) * num45s));

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = r * Math.cos(robotAngle);

        //sets target position
        wheelFL.setTargetPosition(v1 > 0 ? target : -1 * target); //v1 > 0 ? target : -1 * target
        wheelFR.setTargetPosition(v2 > 0 ? target : -1 * target);
        wheelBL.setTargetPosition(v3 > 0 ? target : -1 * target);
        wheelBR.setTargetPosition(v4 > 0 ? target : -1 * target);

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

        telemetry.addData("power ", "%.1f power", v1);
        telemetry.addData("power ", "%.1f power", v2);
        telemetry.addData("power ", "%.1f power", v3);
        telemetry.addData("power ", "%.1f power", v4);
        telemetry.update();

        while(wheelFL.isBusy()  ||  wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();


        }
//        wheelFL.setPower(v1);
//        wheelFR.setPower(v2);
//        wheelBL.setPower(v3);
//        wheelBR.setPower(v4);
    }

    public void travelUntilDistanceAway(int angle, Rev2mDistanceSensor sensor, int distanceInCm) {
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        angle -= 90;

        double r = Math.hypot(Math.cos(Math.toRadians(angle)), Math.sin(Math.toRadians(angle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))) - Math.PI / 4;
        double rightX = 0;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;


        wheelFL.setPower(v1 * 0.5);
        wheelFR.setPower(v2 * 0.5);
        wheelBL.setPower(v3 * 0.5);
        wheelBR.setPower(v4 * 0.5);

        while (true) {
            if (sensor.getDistance(DistanceUnit.CM) <= distanceInCm) {
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                break;
            }
        }
    }

    public void turnSusan (boolean blue, int power){
        if (blue){
            lazyS.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            lazyS.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        lazyS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lazyS.setTargetPosition(700); //v1 > 0 ? target : -1 * target

        lazyS.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lazyS.setVelocity(power);

        while(lazyS.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();


        }

    }


    public void secondLevel(){ //2nd level
        this.targetPosition = armLevel2nd;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
    //    buttonPressed = "X";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void thirdLevel(){  //3rd level
        this.targetPosition = armLevel3rd;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
//        buttonPressed = "Y";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void floorArm(){  //on floor
        this. targetPosition = armLevelFloor;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
//        buttonPressed = "A";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void armLevelForDrive(){  //riding
        this.targetPosition = armLevelRide;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
//        buttonPressed = "B";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void extendArm(int target){
        this.targetPosition += target;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
//        buttonPressed = "DPad_Up";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void retractArm(int target) {
        this.targetPosition -= target;
        motorArm.setTargetPosition(this.targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
//        buttonPressed = "Dpad_Down";
        if (!armMagnet.getState()){ //not sure why it's backwards
            motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    /*
    route 1 = blue side, closest to alliance storage
    route 2 = blue side, far from alliance storage
    route 3 = red side, close
    route 4 = red side, far
    anything else = extra test routes
     travel('', '', 1250) = Leaving starting area
     travel('', '', 1050) = moves one entire tile forward
     turn('', 1700) = 180 degree turn
     turn('', 850) = 90 degree turn
     turn(0, '') = left turn
     turn(180, '') = right turn
     */
    public void route (int route){
        targetPosition = 0;
//        travel(0, 1400, -1250);
//        travel(90, 1400, 1050);
//        turn(0, 1400, 850);
//        travel(270, 1400, 1250);
        extendArm(200);
        travel(0, 700, -500);
        travelUntilDistanceAway(90, distanceR, 35);
        turn(180, 1050, 425);
        travel(90, 700, 470);
        turnSusan(true, 350);
        travel(270, 700, 470);
        turn(0, 1050, 425);
        travel(0, 700, -420);
        turn(0, 1050, 850);
        travel(0, 3000, -4700);
        //lazy susan thing
        retractArm(200);

    }


}
