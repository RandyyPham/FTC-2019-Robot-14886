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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MyRobot {
    /* Public OpMode members. */
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor armMotor;
    DcMotor legMotor;
    Servo hookServo1;
    Servo hookServo2;
    Servo dropServo;
    //ColorSensor colorSensor;
    /* local OpMode members. */
    public HardwareMap hwMap;

    /* Constructor */
    public MyRobot() {
        // this is where all of the code that makes this class.
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        legMotor.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        legMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Drive(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void Stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void TurnLeft(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }

    public void TurnRight(double power) {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }

    public void ArmPower(double power) {
        armMotor.setPower(power);
    }

    public void Unhook() {
        hookServo1.setPosition(0);
        hookServo2.setPosition(1);
    }

    public void Hook() {
        hookServo1.setPosition(1);
        hookServo2.setPosition(0);
    }

    public void Drop() {
        dropServo.setPosition(0);
    }
}

