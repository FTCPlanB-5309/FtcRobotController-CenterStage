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
        if (rightDistance < robot.PROP_THRESHOLD){
            propLocation = propLocation.RIGHT;
        }
        telemetry.addData("leftDistance: ",leftDistance);
        telemetry.addData("rightDistance: ",rightDistance);
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
                drive.forward(10,.2);
                drive.backward(14, .4);
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.backward(10, .2);
                robot.armMotor.setTargetPosition(0);
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

