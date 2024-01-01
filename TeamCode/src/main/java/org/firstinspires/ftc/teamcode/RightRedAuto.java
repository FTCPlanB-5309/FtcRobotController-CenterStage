package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BOARD_DISTANCE;
import static org.firstinspires.ftc.teamcode.RobotHardware.ONE_PIXEL_BOARD_DISTANCE;

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
        Claws claws = new Claws(robot, telemetry, this);
        PropLocation propLocation;
        double back_distance;

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

            switch(propLocation){
                case LEFT: //same as leftblueright
                    strafe.right(15, .2);
                    drive.backward(5, .2);
                    gyroTurn.goodEnough(90);
                    Thread.sleep(1000);
                    drive.forward(35, .2);
                    drive.backward(15, .2);
                    gyroTurn.goodEnough(45);
                    claws.LeftClawOpen();
                    Thread.sleep(500);
                    robot.armServo.setPosition(robot.SHORT_ARM);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    drive.backward(78, .2);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    if (back_distance > BOARD_DISTANCE){
                        drive.backward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                    if (back_distance < BOARD_DISTANCE){
                        drive.forward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                    strafe.right(9, .2);
                    claws.RightClawOpen();
                    drive.forward(5, .2);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.right(55, .2);
                    break;

                case CENTER:
                    drive.forward(10, .2);
                    drive.backward(14, .4);
                    claws.LeftClawOpen();
                    Thread.sleep(500);
                    robot.armServo.setPosition(robot.SHORT_ARM);
                    drive.backward(7, .4);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    drive.backward(78, .2);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    if (back_distance > BOARD_DISTANCE){
                        drive.backward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                    if (back_distance < BOARD_DISTANCE){
                        drive.forward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                    claws.RightClawOpen();
                    Thread.sleep(200);
                    drive.forward(10, .2);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(60, .2);
                    break;

                case RIGHT: //same a leftblueleft
                    strafe.right(37, .2);
                    drive.backward(20, .2);
                    claws.LeftClawOpen();
                    Thread.sleep(500);
                    robot.armServo.setPosition(robot.SHORT_ARM);
                    drive.backward(18, .2);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    drive.backward(33, .2);
                    strafe.right(6, .2);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    if (back_distance > BOARD_DISTANCE){
                        drive.backward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                    if (back_distance < BOARD_DISTANCE){
                        drive.forward (Math.abs((int)back_distance- robot.BOARD_DISTANCE), .2);
                    }
                        claws.RightClawOpen();
                        Thread.sleep(500);
                        drive.forward(14, .2);
                        robot.wristServo.setPosition(robot.GRAB_WRIST);
                        robot.armMotor.setTargetPosition(0);
                        strafe.left(45, .2);
                        break;
                    }


                robot.armMotor.setTargetPosition(robot.ARM_RESET);
                drive.backward(10, .2);
                robot.armServo.setPosition(robot.SHORT_ARM);
                Thread.sleep(30000);


            }
        }


