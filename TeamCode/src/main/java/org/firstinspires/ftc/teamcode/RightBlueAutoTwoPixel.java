package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RightBlueAutoTwoPixel")

public class RightBlueAutoTwoPixel extends LinearOpMode {
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
        robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(90, .25);
        int leftDistance = (int) readSensor.distance(robot.leftDistanceSensor);
        int rightDistance = (int) readSensor.distance(robot.rightDistanceSensor);
        propLocation = findProp.FindPropBackward();


        switch (propLocation) {
            case LEFT: //same as right leftredautotwopixel
                strafe.left(15, .25);
                gyroTurn.goodEnough(89);
                strafe.left(20, .25);
                drive.forward(35,.25);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(20,.25);
                robot.intakeMotor.setPower(0);
                strafe.left(65,.25);
                gyroTurn.goodEnough(-90);
                //2nd Pixel Scoring
                drive.backward(190, .75);
                robot.wristServo.setPosition(robot.WRIST_SCORE_TWO_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.right(72, .35);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case CENTER: //center is the same turns are different
                drive.backward(8, .25);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(29,.25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(89);
                Thread.sleep(300);
                //2nd pixel scoring
                drive.backward(190, .5);
                robot.wristServo.setPosition(robot.WRIST_SCORE_TWO_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                gyroTurn.goodEnough(88);
                Thread.sleep(500);
                strafe.right(51, .25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case RIGHT: //same as left leftredautotwopixel
                strafe.left(33, .25);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                drive.backward(36, .25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                Thread.sleep(300);
                //2nd Pixel Scoring
                drive.backward(218, .5);
                robot.wristServo.setPosition(robot.WRIST_SCORE_TWO_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.right(36, .25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;
        }
        robot.wristServo.setPosition(robot.GRAB_WRIST);
        robot.armMotor.setTargetPosition(robot.ARM_RESET);
    }
}
