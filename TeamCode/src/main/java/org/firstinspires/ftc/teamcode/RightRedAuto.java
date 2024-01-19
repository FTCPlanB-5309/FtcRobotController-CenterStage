package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BOARD_DISTANCE;
import static org.firstinspires.ftc.teamcode.RobotHardware.ONE_PIXEL_BOARD_DISTANCE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "RightRedAuto")
    public class RightRedAuto extends LinearOpMode {
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
            robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
            waitForStart();
            robot.wristServo.setPosition(robot.UPWARDS_WRIST);
            drive.forward(71, .25);
            Thread.sleep(500);
            propLocation = findProp.FindPropForward();

            switch(propLocation){
                case LEFT: //same as leftblueright
                    strafe.right(15, .25);
                    drive.backward(5, .25);
                    gyroTurn.goodEnough(90);
                    Thread.sleep(500);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(500);
                    drive.forward(35, .25);
                    robot.intakeMotor.setPower(-.25);
                    drive.backward(15, .25);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(72, .25);
                    robot.intakeMotor.setPower(0);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    drive.move_to_backboard(back_distance);
                    strafe.right(9, .25);
                    claws.RightClawOpen();
                    drive.forward(5, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(75, .3);
                    break;

                case CENTER:
                    robot.intakeMotor.setPower(-.25);
                    drive.forward(15, .25);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(300);
                    drive.backward(28, .25);
                    robot.intakeMotor.setPower(0);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(72, .25);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    drive.move_to_backboard(back_distance);
                    claws.RightClawOpen();
                    Thread.sleep(200);
                    drive.forward(10, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(65, .25);
                    break;

                case RIGHT: //same a leftblueleft
                    robot.intakeMotor.setPower(-.25);
                    strafe.right(24, .25);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(300);
                    drive.backward(20, .25);
                    Thread.sleep(500);
                    drive.backward(18, .25);
                    robot.intakeMotor.setPower(0);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(39, .25);
                    strafe.right(6, .25);
                    back_distance = (int) readSensor.distance(robot.rearDistanceSensor);
                    drive.move_to_backboard(back_distance);
                    claws.RightClawOpen();
                    Thread.sleep(500);
                    drive.forward(14, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(40, .3);
                    break;
                    }

                robot.armMotor.setTargetPosition(robot.ARM_RESET);
                drive.backward(10, .2);
                robot.armServo.setPosition(robot.SHORT_ARM);
            }
        }


