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
        drive.backward(70, .45);
        drive.backward(20, .25);
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        telemetry.addData("leftDistance: ", leftDistance);
        telemetry.addData("rightDistance: ", rightDistance);
        telemetry.update();

        //left not completed
        if (leftDistance < robot.PROP_THRESHOLD) {
            strafe.left(15, .2);
            drive.backward(5, .2);
            gyroTurn.goodEnough(-90);
            Thread.sleep(500);
            drive.forward(30, .2);
            drive.backward(12, .2);
            gyroTurn.goodEnough(-45);
            claws.LeftClawOpen();
            Thread.sleep(250);
            drive.backward(5, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
        }

        //right (same as RightRedLEFT) not completed
        else if (rightDistance < robot.PROP_THRESHOLD) {
            strafe.left(37, .2);
            drive.backward(16, .2);
            claws.LeftClawOpen();
            drive.backward(10, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
        }

        //middle ROBOT IS STARTING BACKWARDS
        else {
            drive.backward(34, .2);
            claws.LeftClawOpen();
            Thread.sleep(300);
            drive.backward(10, .2);
            gyroTurn.goodEnough(-90);
            drive.backward(180, .3);
            robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
            robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
            robot.armServo.setPosition(robot.SHORT_ARM);
            strafe.left(67, .2);
        }

        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);


    }

}
