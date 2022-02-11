package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmControl extends Testing_ArmTic{

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

    public ArmControl(DcMotorEx motorArm, DigitalChannel armMagnet){
        this.armZeroOverride = false;
        this.armLevelFloor = 10;
        this.armLevelRide = 500;
        this.armLevelCanTilt = 1340;
        this.armLevel2nd = 1015;
        this.armLevel3rd = 1650;
        this.targetPosition = 0;
        this.buttonPressed = "";

        //set up arm motot
        this.motorArm = motorArm;
        motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);

        //set up magnet limit switch
        this.armMagnet = armMagnet;
        armMagnet.setMode(DigitalChannel.Mode.INPUT);
    }


    public String armCheckInput(){

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
            buttonPressed = 1111    "";
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

        return buttonPressed;

    }


    //returns true if arm is above the level where it can tilt
    public boolean canArmTilt(){
        return motorArm.getCurrentPosition() >= armLevelCanTilt;
    }


}
