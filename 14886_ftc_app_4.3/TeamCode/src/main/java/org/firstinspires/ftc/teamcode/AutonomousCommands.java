package org.firstinspires.ftc.teamcode;

public class AutonomousCommands {
    MyRobot robot = new MyRobot();

    public AutonomousCommands() {

    }

    public void Drive(double leftPower, double rightPower) {
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }
}
