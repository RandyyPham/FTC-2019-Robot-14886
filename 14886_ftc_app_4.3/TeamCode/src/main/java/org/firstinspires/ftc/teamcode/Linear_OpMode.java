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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Linear_OpMode", group = "Linear Opmode")
public class Linear_OpMode extends LinearOpMode {

    // Declare OpMode members.
    MyRobot robot = new MyRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double hookServoPosition = 0;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        robot.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        robot.armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        robot.legMotor = hardwareMap.get(DcMotor.class, "leg_motor");
        robot.hookServo = hardwareMap.get(Servo.class, "hook_servo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.armMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.legMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.hookServo.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for the different power values of each drive motor
            double moveValue = gamepad1.left_stick_y;
            double turnValue = gamepad1.right_stick_x;

            double legUp = gamepad1.left_trigger;
            double legDown = gamepad1.right_trigger;

            boolean armUp = gamepad1.left_bumper;
            boolean armDown = gamepad1.right_bumper;
            boolean hookUp = gamepad1.dpad_up;
            boolean hookDown = gamepad1.dpad_down;

            // these variables are for powers to the motor
            double leftPower = Range.clip(moveValue + turnValue, -1.0, 1.0);
            double rightPower = Range.clip(moveValue - turnValue, -1.0, 1.0);
            double legPower = Range.clip(legUp - legDown, -1.0, 1.0);
            double armPower;

            // Drive code here
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            // Arm code here
            if (armUp) {
                armPower = 1.0;
                robot.armMotor.setPower(armPower);
            } else {
                robot.armMotor.setPower(0);
            }

            if (armDown) {
                armPower = -1.0;
                robot.armMotor.setPower(armPower);
            } else {
                robot.armMotor.setPower(0);
            }

            // Leg code here
            robot.legMotor.setPower(legPower);

            // Hook Servo Code Here
            robot.hookServo.setPosition(hookServoPosition);

            if (hookUp && hookServoPosition <= 180) {
                hookServoPosition += .25;
            }

            if (hookDown && hookServoPosition >= 0) {
                hookServoPosition -= .25;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("OwO", null);
            telemetry.update();
        }
    }
}
