package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

    @Autonomous(name = "RightRedAuto")
    public class RightRedAuto extends LinearOpMode {
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
            robot.armMotor.setTargetPosition(ARM_PIXEL_DROP);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(-1);
            drive.forward(71, .25);
            double leftDistance = readSensor.distance(robot.leftDistanceSensor);
            double rightDistance = readSensor.distance(robot.rightDistanceSensor);
            telemetry.addData("leftDistance: ",leftDistance);
            telemetry.addData("rightDistance: ",rightDistance);
            telemetry.update();

            //left (same as leftblueRIGHT) left and right will be flipped
            if(leftDistance < robot.PROP_THRESHOLD){
                strafe.right(15, .2);
                drive.backward(5,.2);
                gyroTurn.goodEnough(90);
                Thread.sleep(1000);
                drive.forward(35,.2);
                drive.backward(17,.2);
                gyroTurn.goodEnough(45);
                claws.LeftClawOpen();
                Thread.sleep(250);
                gyroTurn.goodEnough(90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(83,.2);
                strafe.right(9,.2);
                claws.RightClawOpen();
                drive.forward(5, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(55, .2);
            }

            //right (same as leftblueLEFT)
            else if(rightDistance < robot.PROP_THRESHOLD){
                strafe.right(37,.2);
                drive.backward(20,.2);
                claws.LeftClawOpen();
                Thread.sleep(250);
                drive.backward(12,.2);
                gyroTurn.goodEnough(90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(33,.2);
                drive.backward(12,.1);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(14, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.left(45, .2);
            }

            //middle - remember the lefts and rights need to be switched
            else{
                drive.forward(10,.2);
                drive.backward(14, .4);
                claws.LeftClawOpen();
                Thread.sleep(300);
                drive.backward(4,.4);
                gyroTurn.goodEnough(90);
                robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armServo.setPosition(robot.SHORT_ARM);
                drive.backward(80,.2);
                claws.RightClawOpen();
                Thread.sleep(200);
                drive.forward(10, .2);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.left(60, .2);
            }

            robot.armMotor.setTargetPosition(robot.ARM_RESET);
            drive.backward(10,.2);
            robot.armServo.setPosition(robot.SHORT_ARM);
            Thread.sleep(30000);


        }
    }

