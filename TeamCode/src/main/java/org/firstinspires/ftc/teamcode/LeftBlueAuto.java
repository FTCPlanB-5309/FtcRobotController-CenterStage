package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BOARD_DISTANCE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftBlueAuto")
public class LeftBlueAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    Claws claws = new Claws(robot, telemetry, this);
    PropLocation propLocation;
    int back_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        waitForStart();
        robot.armServo.setPosition(robot.LONG_ARM);
        robot.wristServo.setPosition(robot.WRIST_DROP_PIXEL);
        Thread.sleep(100);
        robot.armMotor.setTargetPosition(ARM_PIXEL_DROP);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(-1);
        Thread.sleep(100);
        drive.forward(71, .25);
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        propLocation = propLocation.CENTER;
        if (leftDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.LEFT;
        }
        if (rightDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.RIGHT;
        }
        telemetry.addData("leftDistance: ", leftDistance);
        telemetry.addData("rightDistance: ", rightDistance);
        telemetry.update();

        switch (propLocation) {
            case LEFT:
                strafe.left(24, .2);
                drive.backward(16, .2);
                claws.LeftClawOpen();
                Thread.sleep(500);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(8, .2);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                drive.backward(58, .2);
                strafe.left(7,.25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(14, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(55, .3);
                break;

            case CENTER:
                drive.forward(10, .2);
                drive.backward(14, .4);
                claws.LeftClawOpen();
                Thread.sleep(300);
                drive.backward(8, .4);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(73, .2);
                strafe.left(10,.25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(10, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(60, .3);
                break;

            case RIGHT:
                strafe.left(15, .2);
                drive.backward(5, .2);
                gyroTurn.goodEnough(-90);
                Thread.sleep(1000);
                drive.forward(35, .2);
                drive.backward(15, .2);
                claws.LeftClawOpen();
                Thread.sleep(500);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(72, .2);
                strafe.left(20, .2);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(5, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(87, .3);
                break;
        }

        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        drive.backward(10, .2);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);

    }
}








