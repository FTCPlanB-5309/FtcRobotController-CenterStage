package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

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
    Claws claws = new Claws(robot,telemetry,this);

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
        telemetry.addData("leftDistance: ",leftDistance);
        telemetry.addData("rightDistance: ",rightDistance);
        telemetry.update();

        //left
        if(leftDistance < robot.PROP_THRESHOLD){
            strafe.left(19,.2);
            drive.backward(16,.2);
            claws.LeftClawOpen();
            drive.backward(4,.2);
            gyroTurn.goodEnough(-90);
            robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
            robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
            robot.armServo.setPosition(robot.SHORT_ARM);
            drive.backward(63,.2);
            claws.RightClawOpen();
            Thread.sleep(500);
            drive.forward(14, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
            strafe.right(55, .2);


        }
        //right
        else if(rightDistance < robot.PROP_THRESHOLD){
            strafe.left(15, .2);
            drive.backward(5,.2);
            gyroTurn.goodEnough(-90);
            Thread.sleep(1000);
            drive.forward(35,.2);
            drive.backward(17,.2);
            claws.LeftClawOpen();
            Thread.sleep(500);
            robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
            robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
            robot.armServo.setPosition(robot.SHORT_ARM);
            drive.backward(78,.2);
            strafe.left(9,.2);
            claws.RightClawOpen();
            drive.forward(5, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
            strafe.left(55, .2);
        }

        //middle
        else{
            drive.forward(10,.2);
            drive.backward(14, .4);
            claws.LeftClawOpen();
            Thread.sleep(300);
            drive.backward(4,.4);
            gyroTurn.goodEnough(-90);
            robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
            robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
            robot.armServo.setPosition(robot.SHORT_ARM);
            drive.backward(80,.2);
            claws.RightClawOpen();
            Thread.sleep(200);
            drive.forward(10, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
            strafe.right(60, .2);

        }

        drive.backward(10,.2);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);


    }
}

