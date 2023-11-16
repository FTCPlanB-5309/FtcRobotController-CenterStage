package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    DriveTrain driveTrain;

    public Drive(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.driveTrain = new DriveTrain(robot, telemetry,linearOpMode);
    }

    public void forward (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(speed);
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }

    public void backward (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * -distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(-speed);
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }
    public void backward_auto (int distance, double speed, int howHigh) throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * -distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(-speed);

        while ((driveTrain.isBusy() && linearOpMode.opModeIsActive())){
            Thread.yield();
        }


        driveTrain.stop();
    }

    public void forward_auto (int distance, double speed, int howHigh) throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(speed);

        while ((driveTrain.isBusy() && linearOpMode.opModeIsActive())){
            Thread.yield();
        }


        driveTrain.stop();
    }
//
//    public void find_prop (int distance, double speed, int howHigh) throws InterruptedException {
//        if (!linearOpMode.opModeIsActive())
//            return;
//        driveTrain.stop_and_reset_enconders();
//        driveTrain.run_using_encoder();
//        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * -distance);
//        driveTrain.run_to_position();
//        driveTrain.setSpeed(-speed);
//
//
//
//        while ((driveTrain.isBusy() && linearOpMode.opModeIsActive())){
//            Thread.yield();
//            if (robot.rightDistance > robot.rightDistanceSensor.getDistance(DistanceUnit.CM)){
//                robot.rightDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);
//            }
//            if (robot.leftDistance > robot.leftDistanceSensor.getDistance(DistanceUnit.CM)){
//                robot.leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
//            }
//        }
//
//        driveTrain.stop();
//    }
}
