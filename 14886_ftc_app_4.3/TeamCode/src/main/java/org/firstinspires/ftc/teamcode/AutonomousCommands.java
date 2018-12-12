package org.firstinspires.ftc.teamcode;

public class AutonomousCommands {
    MyRobot robot = new MyRobot();

    public AutonomousCommands() {

    }

    public void Drive(double power) {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
    }

    public void Stop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void DropMarker(double position) {
        robot.markerServo.setPosition(position);
    }
    
    public void TurnLeft(double power){
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
    }

    public void TurnRight(double power){
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
    }

}
