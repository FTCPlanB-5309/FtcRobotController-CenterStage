package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

    @Autonomous(name = "RightBlueAuto")

    public class RightBlueAuto extends LinearOpMode {
        RobotHardware robot = new RobotHardware();
        Drive drive = new Drive(robot, telemetry, this);
        Strafe strafe = new Strafe(robot, telemetry, this);
        GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
        ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
        Claws claws = new Claws(robot,telemetry,this);
        PropLocation propLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(90, .25);
        int leftDistance = (int) readSensor.distance(robot.leftDistanceSensor);
        int rightDistance = (int) readSensor.distance(robot.rightDistanceSensor);
        propLocation = propLocation.CENTER;
        if (leftDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.LEFT;
        }
        if (rightDistance < robot.PROP_THRESHOLD){
            propLocation = propLocation.RIGHT;
        }
        telemetry.addData("leftDistance: ", leftDistance);
        telemetry.addData("rightDistance: ", rightDistance);
        telemetry.update();

        switch (propLocation){
            case LEFT:
                strafe.right(15, .2);
                drive.backward(5,.2);
                gyroTurn.goodEnough(90);
                Thread.sleep(500);
                drive.forward(30,.2);
                drive.backward(12,.2);
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.backward(5,.2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                break;

            case CENTER:
                robot.intakeMotor.setPower(-.25);
                drive.backward(8, .25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                drive.backward(34, .25);
                Thread.sleep(300);
                /*
                gyroTurn.goodEnough(90);
                robot.intakeMotor.setPower(0);
                drive.backward(180, .3);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setPower(-1);
                strafe.right(67, .2);
                Thread.sleep(100000);
                leftDistance = (int)readSensor.distance(robot.leftDistanceSensor);
                if (leftDistance<robot.RED_CENTER_DISTANCE){
                    strafe.right(robot.RED_CENTER_DISTANCE - leftDistance,.2);
                }
                if (leftDistance>robot.RED_CENTER_DISTANCE){
                    strafe.left(leftDistance - robot.RED_CENTER_DISTANCE,.2);
                }
                int rearDistance = (int) readSensor.distance(robot.rearDistanceSensor);
                if (rearDistance>robot.BOARD_DISTANCE){
                    drive.backward(rearDistance - robot.BOARD_DISTANCE,.2);
                }*/
                break;

            case RIGHT:
                strafe.right(37,.2);
                drive.backward(16,.2);
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.backward(10, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                break;
        }

        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);


    }
}

