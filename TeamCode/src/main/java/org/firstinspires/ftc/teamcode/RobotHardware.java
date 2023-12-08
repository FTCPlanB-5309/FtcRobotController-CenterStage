/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardware {

    // Create Motors
    public DcMotor backLeftMotor   = null;
    public DcMotor backRightMotor  = null;
    public DcMotor frontLeftMotor   = null;
    public DcMotor frontRightMotor = null;
    public DcMotor armMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor droneMotor = null;

    // Motor Constants
    public static final int ARM_PIXEL_DROP = -530;
    public static final int ARM_READY = -2321;
    public static final int ARM_UP = -1309;
    public static final int ARM_PIXEL_SCORE = 6908;

    //Create Odometry Motors
//    public DcMotor leftOdometry = null;
//    public DcMotor middleOdometry = null;
//    public DcMotor rightOdometry = null;

    //Create Servos
    public Servo wristServo = null;
    public Servo leftClawServo = null;
    public Servo rightClawServo = null;
    public Servo armServo = null;
    public Servo hookServo = null;
    public Servo leftPixelLockServo = null;
    public Servo rightPixelLockServo = null;

    //Create Sensors
    BNO055IMU imu;
    public Rev2mDistanceSensor rightDistanceSensor = null;
    public Rev2mDistanceSensor leftDistanceSensor = null;

    //Servo Constants
    //Pixel Locks
    public static final double LEFT_PIXEL_UNLOCK = .75;
    public static final double LEFT_PIXEL_LOCK = .25;
    public static final double RIGHT_PIXEL_UNLOCK = .25;
    public static final double RIGHT_PIXEL_LOCK = .75;
    //Claws
    public static final double LEFT_CLAW_CLOSE = 0.85;
    public static final double LEFT_CLAW_OPEN = 0.55;
    public static final double RIGHT_CLAW_CLOSE = 0.75;
    public static final double RIGHT_CLAW_OPEN = 0.25;
    //Wrist
    public static final double UPWARDS_WRIST = .75;
    public static final double RESTING_WRIST = .619;
    public static final double GRAB_WRIST = .49;
    public static final double WRIST_DROP_PIXEL = .33;
    public static final double WRIST_SCORE_PIXEL = .73;
    //Hook
    public static final double HOOK_IN = .75;
    public static final double HOOK_OUT = .4;
    //Arm
    public static final double SHORT_ARM = .89;
    public static final double GRAB_ARM = .79;
    public static final double LONG_ARM = .1;
    //Rates
    public static final double WRIST_SERVO_CHANGE_RATE = .004;
    public static final double ARM_SERVO_CHANGE_RATE = .003;
    public static final int CLICKS_PER_CENTIMETER = 18;
    public static final double ODOMETRY_CLICKS_PER_CENTIMETER = 362.165;
    public static final int STRAFE_CLICKS_PER_CENTIMETER = 20;

    //Sensor Values
    public double leftDistance = 8192;
    public double rightDistance = 8192;
    public static final double PROP_THRESHOLD = 12;

    //Turning Speeds
    public final double HIGH_TURN_POWER = 0.6;
    public final double LOW_TURN_POWER = 0.07;

    //Odometry Values
    final static double L = 17.531;//distance between encoder 1 and 2 in cm
    final static double B = 35.062;//distance between the midpoint of encoder 1 and 2 and encoder 3
    final static double R = 3.6;//wheel radius in cm
    final static double N = 8192; //encoder ticks per revolution, REV encoder
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    //Keep Track of Odometry Encoders Between Updates
    public int currentRightPosition = 0;
    public int currentMiddlePosition = 0;
    public int currentLeftPosition = 0;

    private int oldRightPosition = 0;
    private int oldMiddlePosition = 0;
    private int oldLeftPosition = 0;

    //XyhVector is a tuple (x,y,h) where h is the heading of the robot
    //We need to write the code for XyhVector
//    public XyhVector START_POS = new XyhVector(91.44, 210.82, Math.toRadians(90));
//    public XyhVector pos = new XyhVector(START_POS);


    /* local OpMode members. */
    public HardwareMap hwMap           =  null;



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        backLeftMotor  = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        droneMotor = hwMap.get(DcMotor.class, "droneMotor");

        // Odometry
//        leftOdometry = hwMap.get(DcMotor.class, "leftOdometry");
//        middleOdometry = hwMap.get(DcMotor.class, "frontLeftMotor");
//        rightOdometry = hwMap.get(DcMotor.class, "frontRightMotor");



        wristServo = hwMap.get(Servo.class, "wristServo");
        leftClawServo = hwMap.get(Servo.class, "leftClawServo");
        rightClawServo = hwMap.get(Servo.class, "rightClawServo");
        armServo = hwMap.get(Servo.class, "armServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        rightPixelLockServo = hwMap.get(Servo.class, "rightPixelLockServo");
        leftPixelLockServo = hwMap.get(Servo.class, "leftPixelLockServo");

        leftDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        droneMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Odometry
//        leftOdometry.setDirection(DcMotorSimple.Direction.REVERSE);

        //Using Encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        droneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        middleOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        middleOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    public void auto_init() {
        // GyroTurn initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Initalize Servos
        rightClawServo.setPosition(RIGHT_CLAW_CLOSE);
        leftClawServo.setPosition(LEFT_CLAW_CLOSE);
        armServo.setPosition(SHORT_ARM);
        hookServo.setPosition(HOOK_IN);
    }
//    public void odometry(){
//
//        oldRightPosition = currentRightPosition;
//        oldMiddlePosition = currentMiddlePosition;
//        oldLeftPosition = currentLeftPosition;
//
//        currentRightPosition = -frontRightMotor.getCurrentPosition();
//        currentMiddlePosition = frontLeftMotor.getCurrentPosition();
//        currentLeftPosition = -leftOdometry.getCurrentPosition();
//
//        int dn2 = currentRightPosition - oldRightPosition;
//        int dn3 = currentMiddlePosition - oldMiddlePosition;
//        int dn1 = currentLeftPosition - oldLeftPosition;
//
//        //The robot has moved and turned a tiny bit between the two measurements
//        double dtheta = cm_per_tick * (dn2-dn1) / L;
//        double dx = cm_per_tick * (dn1+dn2) / 2.0;
//        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);
//
//        //Small movement of the robot gets added to the filed coordinate system
//        double theta = pos.h + (dtheta / 2.0);
//        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
//        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
//        pos.h += dtheta;
//
//        //Limit theta to +/- PI or +/- 180 degrees
//        pos.h = pos.h % (2.0 * Math.PI);
//        if (pos.h > Math.PI){
//            pos.h -= 2.0 * Math.PI;
//        }
//        if (pos.h < -Math.PI ){
//            pos.h += 2.0 * Math.PI;
//        }
//    }
}
