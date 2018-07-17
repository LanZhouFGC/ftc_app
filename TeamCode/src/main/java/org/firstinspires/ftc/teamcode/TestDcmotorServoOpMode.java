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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "TestDcmotorServoOpMode", group = "Test")
//@Disabled
public class TestDcmotorServoOpMode extends OpMode {
    double startpos_blue = 1.0;
    double startpos_orange = 0.15;
    double startpos_turntable = 0.50;
    double endpos_o_turntable = 0.00;
    double endpos_b_turntable = 1.00;
    double endpos_orange = 1.00;
    double endpos_blue = 0.25;
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
    private ColorSensor sensorColor = null;//
    private enum Color{
        UNKNOWN,EMPTY,RED,BLUE,YELLOW;
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
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
        motorhangr.setDirection(DcMotor.Direction.REVERSE);
        //motor direction
        motorhangr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorhangl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor behavior
        sensorColor = hardwareMap.colorSensor.get("color");
        //
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {

    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
        servoorangedoor.setPosition(0.15);
        servoturntable.setPosition(0.50);
        servobluedoor.setPosition(1.00);
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("________________________________");
        telemetry.addLine("!!!!!!!!!!!!!clolrsensor!!!!!!!!");
        printColorSensorInfo(sensorColor);
        telemetry.addLine("!!!!!!GetColor!!!!!!");
        telemetry.addLine(getColor(sensorColor).toString());
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!LF!!!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorLeft1);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!LB!!!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorLeft2);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!!RF!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorRight1);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!!RB!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorRight2);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!!RH!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorhangr);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("!!!!!!!!!!!!!!LH!!!!!!!!!!!!!!!");
        printDcMotorInfo(motorhangl);
        telemetry.addLine("*******************************");
        telemetry.addLine("********************************");
        telemetry.addLine("servobluedoor");
        printServoInfo(servobluedoor);
        telemetry.addLine("servoorangedoor");
        printServoInfo(servoorangedoor);
        telemetry.addLine("servoturntable");
        printServoInfo(servoturntable);
       /* servobluedoor.setPosition(gamepad1.left_stick_y);
        servoorangedoor.setPosition(gamepad1.right_stick_y);
        servoturntable.setPosition(gamepad2.right_stick_y);*/
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

    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {

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

    private void printDcMotorInfo(DcMotor motor) {
        telemetry.addData("DeviceName", motor.getDeviceName());
        telemetry.addData("Manufacturer", motor.getManufacturer());
        telemetry.addData("Version", motor.getVersion());
        telemetry.addData("ConnectionInfo", motor.getConnectionInfo());
        telemetry.addData("MotorType", motor.getMotorType());
        telemetry.addData("Controller", motor.getController());
        telemetry.addData("PortNumber", motor.getPortNumber());
        telemetry.addData("PowerFloat", motor.getPowerFloat());
        telemetry.addData("ZeroPowerBehavior", motor.getZeroPowerBehavior());
        telemetry.addData("Mode", motor.getMode());
        telemetry.addData("Direction", motor.getDirection());
        telemetry.addData("CurrentPosition", motor.getCurrentPosition());
        telemetry.addData("TargetPosition", motor.getTargetPosition());
        telemetry.addData("Power", motor.getPower());
    }

    private void printServoInfo(Servo servo) {
        telemetry.addData("DeviceName", servo.getDeviceName());
        telemetry.addData("Manufacturer", servo.getManufacturer());
        telemetry.addData("Version", servo.getVersion());
        telemetry.addData("ConnectionInfo", servo.getConnectionInfo());
        telemetry.addData("Controller", servo.getController());
        telemetry.addData("PortNumber", servo.getPortNumber());
        telemetry.addData("Direction", servo.getDirection());
        telemetry.addData("Position", servo.getPosition());
    }

    private void printColorSensorInfo(ColorSensor color) {
        telemetry.addData("I2cAddress", color.getI2cAddress());
        telemetry.addData("alpha", color.alpha());
        telemetry.addData("argb", color.argb());
        telemetry.addData("R", color.red());
        telemetry.addData("G", color.green());
        telemetry.addData("B", color.blue());

    }
    private Color getColor(ColorSensor color)
    {
        double R=color.red();
        double G=color.green();
        double B=color.blue();
        double x;
        double d;
        x=(R+G+B)/3;
        d=(Math.abs(R-x)+Math.abs(G-x)+Math.abs(B-x))/x;
        if (d>0.4){
            //Not empty
            x=(R+B)/2;
            d=(R-B)/x;
            if(d>+0.4){
                //RedOrYellow
                x=(G+B)/2;
                d=(G-B)/x;
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

}