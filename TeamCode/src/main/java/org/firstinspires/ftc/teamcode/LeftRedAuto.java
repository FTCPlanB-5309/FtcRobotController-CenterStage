package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftRedAuto")

public class LeftRedAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    Claws claws = new Claws(robot,telemetry,this);
    PropLocation propLocation;
    FindProp findProp;


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
        drive.forward(67, .25);
        propLocation = findProp.FindPropForward();
        Thread.sleep(1000);

        //left (same as RightRedRIGHT)
        switch (propLocation){
            case LEFT:
                strafe.left(32,.2);
                drive.backward(16,.2);
                claws.LeftClawOpen();
                drive.backward(10, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                break;

            case CENTER:
                drive.forward(14,.2);
                drive.backward(18, .4);
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.backward(10, .2);
                robot.armMotor.setTargetPosition(0);
                break;

            case RIGHT:
                strafe.left(15, .2);
                drive.backward(5,.2);
                gyroTurn.goodEnough(-90);
                Thread.sleep(500);
                drive.forward(30,.2);
                drive.backward(10,.2);
                claws.LeftClawOpen();
                Thread.sleep(750);
                drive.backward(5,.2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                break;
        }

        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);


    }
}