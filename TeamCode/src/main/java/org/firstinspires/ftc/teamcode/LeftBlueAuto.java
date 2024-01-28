package org.firstinspires.ftc.teamcode;


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
    FindProp findProp = new FindProp(robot, telemetry, this);

    PropLocation propLocation;
    int back_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.forward(71, .25);
        Thread.sleep(500);
        propLocation = findProp.FindPropForward();

        switch (propLocation) {
            case LEFT:
                robot.intakeMotor.setPower(-.25);
                strafe.left(24, .2);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(300);
                drive.backward(16, .2);
                Thread.sleep(500);
                drive.backward(8, .2);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setPower(-1);
                drive.backward(58, .2);
                strafe.left(7,.25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(14, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(55, .3);
                break;

            case CENTER:
                robot.intakeMotor.setPower(-.25);
                drive.forward(10, .2);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(300);
                drive.backward(14, .4);
                Thread.sleep(300);
                drive.backward(8, .4);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setPower(-1);
                drive.backward(73, .2);
                strafe.left(10,.25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
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
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                drive.forward(35, .2);
                robot.intakeMotor.setPower(-.25);
                drive.backward(15, .2);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setPower(-1);
                drive.backward(72, .2);
                robot.intakeMotor.setPower(0);
                strafe.left(23, .2);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
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








