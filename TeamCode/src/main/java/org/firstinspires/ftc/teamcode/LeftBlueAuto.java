package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        robot.armMotor.setTargetPosition(ARM_PIXEL_DROP);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(-1);
        drive.forward(77, .25);
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        telemetry.addData("leftDistance: ",leftDistance);
        telemetry.addData("rightDistance: ",rightDistance);
        telemetry.update();
        if(leftDistance < robot.PROP_THRESHOLD){
        drive.backward(10,.2);
        strafe.left(21,.2);
        claws.LeftClawOpen();
        drive.backward(10,.2);
        strafe.left(71,.4);
        }
        else if(rightDistance < robot.PROP_THRESHOLD){
            drive.backward(10,.2);
            gyroTurn.goodEnough(-90);
            drive.forward(10,.2);
            claws.LeftClawOpen();
            Thread.sleep(1000);
            drive.backward(92,.4);
        }
        else{
            drive.backward(10, .4);
            claws.LeftClawOpen();
            Thread.sleep(1000);
            drive.backward(20,.2);
            strafe.left(92,.4);
        }
        Thread.sleep(30000);


    }
}

