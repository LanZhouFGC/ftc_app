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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="2", group="ColorSensorTest")
//@Disabled
public class ColorSensorTest_2_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private double startpos_blue = 1.0;
    private double startpos_orange = 0.15;
    private double startpos_turntable = 0.50;
    private double endpos_o_turntable = 0.00;
    private double endpos_b_turntable = 1.00;
    private double endpos_orange = 1.00;
    private double endpos_blue = 0.25;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeft1 = null;
    private DcMotor motorLeft2 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorRight2 = null;
    private DcMotor motorupd = null;//5
    private DcMotor motorupu = null;//6
    private DcMotor motorhangl = null;//7
    private DcMotor motorhangr = null;//8
    private Servo servoturntable = null;//11
    private Servo servoorangedoor = null; //12
    private Servo servobluedoor = null;//13
    private NormalizedColorSensor sensorColor = null;//
    private enum Color{
        UNKNOWN,EMPTY,RED,BLUE,YELLOW;
    }

    private void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {

        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine
            // control
            // while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }

        }
        motorLeft1.setPower(leftMotorSpeed);
        motorLeft2.setPower(leftMotorSpeed);
        motorRight1.setPower(rightMotorSpeed);
        motorRight2.setPower(rightMotorSpeed);
    }


    private double limit(double input) {
        if (input > 1.0) {
            input = 1.0;
        } else if (input < -1.0) {
            input = -1.0;
        }
        return input;

    }

    private Color getColor(NormalizedColorSensor color)
    {
        double R=color.getNormalizedColors().red;
        double G=color.getNormalizedColors().green;
        double B=color.getNormalizedColors().blue;
        double x;
        double d;
        x=checkZero((R+G+B)/3);
        d=checkZero((Math.abs(R-x)+Math.abs(G-x)+Math.abs(B-x))/x);
        if (d>0.4){
            //Not empty
            x=checkZero((R+B) /2);
            d=checkZero((R-B)/x);
            if(d>+0.4){
                //RedOrYellow
                x=checkZero((G+B)/2);
                d=checkZero((G-B)/x);
                if (d>+0.4){
                    //Yellow
                    return Color.YELLOW;
                }
                else
                    return Color.RED;
            }
            else if(d<-0.4){
                //Blue
                return Color.BLUE;
            }
            //!!!!!!
            else{
                return Color.UNKNOWN;
            }
        }
        else {
            return Color.EMPTY;
        }


    }
    private double checkZero(double x)
    {
        if (x==0.0)
            x=0.0001;
        return x;
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorLeft1 = hardwareMap.dcMotor.get("l1");
        motorLeft2 = hardwareMap.dcMotor.get("l2");
        motorRight1 = hardwareMap.dcMotor.get("r1");
        motorRight2 = hardwareMap.dcMotor.get("r2");
        //chassis drive above
        motorupd = hardwareMap.dcMotor.get("upd");
        motorupu = hardwareMap.dcMotor.get("upu");
        motorhangl = hardwareMap.dcMotor.get("hl");
        motorhangr = hardwareMap.dcMotor.get("hr");
        //functional motor above
        servoturntable = hardwareMap.servo.get("stt");
        servoorangedoor = hardwareMap.servo.get("sod");
        servobluedoor = hardwareMap.servo.get("sbd");
        //functional servo above
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorhangl.setDirection(DcMotor.Direction.REVERSE);
        //motor direction
        motorhangr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorhangl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor behavior
        sensorColor = hardwareMap.get(NormalizedColorSensor.class,"color");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        servoorangedoor.setPosition(startpos_orange);
        servoturntable.setPosition(startpos_turntable);
        servobluedoor.setPosition(startpos_blue);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            arcadeDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, true);
            if (gamepad1.a == true) {
                motorupd.setPower(1.0);
                motorupu.setPower(1.0);
            } else {
                motorupd.setPower(0);
                motorupu.setPower(0);
            }
            if (gamepad2.dpad_left == true) {
                servoturntable.setPosition(endpos_b_turntable);
            } else if (gamepad2.dpad_right == true) {
                servoturntable.setPosition(endpos_o_turntable);
            } else if (gamepad2.right_trigger>0.3){
                switch (getColor(sensorColor)){
                    case RED:
                    case YELLOW:
                        servoturntable.setPosition(endpos_o_turntable);
                        break;
                    case BLUE:
                        servoturntable.setPosition(endpos_b_turntable);
                        break;
                    default:
                        servoturntable.setPosition(startpos_turntable);
                        break;
                }
            }
            else {
                servoturntable.setPosition(startpos_turntable);
            }
            if (gamepad2.x == true) {
                servobluedoor.setPosition(endpos_blue);
            } else {
                servobluedoor.setPosition(startpos_blue);
            }
            if (gamepad2.y == true) {
                servoorangedoor.setPosition(endpos_orange);
            } else {
                servoorangedoor.setPosition(startpos_orange);
            }
            if (gamepad2.left_bumper == true) {
                motorhangl.setPower(-1.0);
                motorhangr.setPower(-1.0);
            } else if (gamepad2.right_bumper == true) {
                motorhangl.setPower(1.0);
                motorhangr.setPower(1.0);
            } else {

                motorhangl.setPower(0.0);
                motorhangr.setPower(0.0);
            }
            if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5) {
                motorLeft1.setPower(-1.0);
                motorRight1.setPower(-1.0);
                motorRight2.setPower(1.0);
                motorLeft2.setPower(1.0);
            }
        }
    }
}
