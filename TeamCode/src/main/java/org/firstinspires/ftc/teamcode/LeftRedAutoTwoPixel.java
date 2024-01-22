package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "LeftRedAutoTwoPixel")

public class LeftRedAutoTwoPixel extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    Claws claws = new Claws(robot, telemetry, this);
    FindProp findProp = new FindProp(robot, telemetry, this);
    PropLocation propLocation;
    int back_distance;
    int side_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(86, .25);
        int leftDistance = (int) readSensor.distance(robot.leftDistanceSensor);
        int rightDistance = (int) readSensor.distance(robot.rightDistanceSensor);
        propLocation = findProp.FindPropBackward();

        //left not completed
        switch (propLocation){
            case LEFT:
                strafe.right(33, .25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
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
                strafe.left(36, .25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case CENTER:
                drive.backward(12, .25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(25,.25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                Thread.sleep(300);
                //2nd pixel scoring
                drive.backward(190, .5);
                robot.wristServo.setPosition(robot.WRIST_SCORE_TWO_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.left(56, .25);
                back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case RIGHT:

                //first part from leftRedAuto
                strafe.right(15, .25);
                gyroTurn.goodEnough(89);
                strafe.right(20, .25);
                drive.forward(35,.25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
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
                strafe.left(72, .35);
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
