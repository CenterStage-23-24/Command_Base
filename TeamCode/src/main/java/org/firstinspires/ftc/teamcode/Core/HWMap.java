package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    // Drive Motors
    private final Motor leftFrontMotor;
    private final Motor leftBackMotor;
    private final Motor rightBackMotor;
    private final Motor rightFrontMotor;
    private final MecanumDrive mecanumDrive;

    // Mechanism Motors
    private final Motor linearSlidesLeft1;
    private final Motor linearSlidesRight1;
    private final Motor linearSlidesRight2;
    private final Motor intakeMotor;
    //IMU
    private static IMU imu;
    public static double imuAngle;

    //Servos

    private final Servo droneLaunchServo;

    private final CRServo gripperAxonServoLeft;
    private final CRServo gripperAxonServoRight;
    private final AnalogInput gripperAxonAnalogLeft;
    private final AnalogInput gripperAxonAnalogRight;
    private final CRServo axonServoLeft;
    private final CRServo axonServoRight;
    private final AnalogInput axonAnalogLeft;
    private final AnalogInput axonAnalogRight;

    private final Servo OdoRetractionLeft;
    private final Servo OdoRetractionRight;
    //private final Servo OdoRetractionMiddle;
    private Rev2mDistanceSensor distanceSensor;
    private Rev2mDistanceSensor distanceSensor2;

    private final RevBlinkinLedDriver leftBlinkin;
    private final RevBlinkinLedDriver rightBlinkin;

    //Sensors

    private final RevColorSensorV3 trayLeftCS;
    private final RevColorSensorV3 trayRightCS;

    private HardwareMap hardwareMap;
    List<LynxModule> hubs;

    public HWMap(HardwareMap hardwareMap, boolean isAuto) {
        hubs = hardwareMap.getAll(LynxModule.class);


        //Drive Motors
        this.hardwareMap = hardwareMap;
        rightFrontMotor = new Motor(hardwareMap, "RF", Motor.GoBILDA.RPM_435); //CH Port 0
        leftFrontMotor = new Motor(hardwareMap, "LF", Motor.GoBILDA.RPM_435);//CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = new Motor(hardwareMap, "LB", Motor.GoBILDA.RPM_435); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = new Motor(hardwareMap, "RB", Motor.GoBILDA.RPM_435);//CH Port 3. The left odo pod accesses this motor's encoder port.

        //Linear Slides Motor
        linearSlidesLeft1 = new Motor(hardwareMap, "LSL", Motor.GoBILDA.BARE); //EH Port 1
        linearSlidesRight1 = new Motor(hardwareMap, "LSR1", Motor.GoBILDA.BARE);//EH Port 2
        linearSlidesRight2 = new Motor(hardwareMap, "LSR2", Motor.GoBILDA.BARE); // EH Port 3

        // Intake Motor
        intakeMotor = new Motor(hardwareMap, "IM", Motor.GoBILDA.RPM_1150); //EH Port 0
        intakeMotor.setInverted(true);

        //Outtake Servos
        gripperAxonServoLeft = new CRServo(hardwareMap, "CAL"); //CH Port 0
        gripperAxonServoRight = new CRServo(hardwareMap, "CAR");//CH Port 1
        gripperAxonAnalogLeft = hardwareMap.get(AnalogInput.class, "CALA");   // CH port 0
        gripperAxonAnalogRight = hardwareMap.get(AnalogInput.class, "CARA"); // CH  port 1
        droneLaunchServo = hardwareMap.get(Servo.class, "DLS");

        //ODO retraction Servos
        OdoRetractionLeft = hardwareMap.get(Servo.class, "ORL"); //CH Port 0
        //OdoRetractionMiddle = hardwareMap.get(Servo.class, "ORM");//CH Port 1
        OdoRetractionRight = hardwareMap.get(Servo.class, "ORR");//CH Port 2

        //Linear Slides Servos
        axonServoLeft = new CRServo(hardwareMap, "ASL");//EH Port 0
        axonServoRight = new CRServo(hardwareMap, "ASR");//EH Port 1
        axonAnalogLeft = hardwareMap.get(AnalogInput.class, "AAL"); //EH Port 0
        axonAnalogRight = hardwareMap.get(AnalogInput.class, "AAR"); //EH Port 2

        axonServoLeft.setInverted(false);//Counterclockwise
        axonServoRight.setInverted(false);//Clockwise


        trayRightCS = hardwareMap.get(RevColorSensorV3.class, "TRCS");//EH Port 1
        trayLeftCS = hardwareMap.get(RevColorSensorV3.class, "TLCS");//EH Port 2
        //Blinkins
        leftBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "LBL");// CH Port 2
        rightBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "RBL");// CH Port 3

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor2");

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        linearSlidesLeft1.setInverted(true);
        linearSlidesRight1.setInverted(true);
        linearSlidesRight2.setInverted(true);
        intakeMotor.setInverted(true);

        //Set Motor Mode
        leftBackMotor.setRunMode(Motor.RunMode.RawPower);
        rightBackMotor.setRunMode(Motor.RunMode.RawPower);
        leftFrontMotor.setRunMode(Motor.RunMode.RawPower);
        rightFrontMotor.setRunMode(Motor.RunMode.RawPower);

        linearSlidesRight1.setRunMode(Motor.RunMode.RawPower);
        linearSlidesLeft1.setRunMode(Motor.RunMode.RawPower);
        linearSlidesRight2.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        //Mecanum Drive Initialization
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        mecanumDrive.setRightSideInverted(false);
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
        if(isAuto){
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            imu = hardwareMap.get(IMU.class, "imu");
            initializeIMU();
            linearSlidesLeft1.resetEncoder();
        } else{
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
    }


    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuAngle;
    }

    public static void initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
    }


    public RevColorSensorV3 getTrayRightCS() {
        return trayRightCS;
    }

    public RevColorSensorV3 getTrayLeftCS() {
        return trayLeftCS;
    }

    public Rev2mDistanceSensor getDistanceSensor() {
        return distanceSensor;
    }

    public Rev2mDistanceSensor getDistanceSensor2(){
        return distanceSensor2;
    }

    public Servo getDroneLaunchServo() {
        return droneLaunchServo;
    }

    public CRServo getGripperAxonServoRight() {
        return gripperAxonServoRight;
    }

    public CRServo getGripperAxonServoLeft() {
        return gripperAxonServoLeft;
    }

    public CRServo getAxonServoRight() {
        return axonServoRight;
    }

    public CRServo getAxonServoLeft() {
        return axonServoLeft;
    }

    public AnalogInput getGripperAxonAnalogLeft() {
        return gripperAxonAnalogLeft;
    }

    public AnalogInput getGripperAxonAnalogRight() {
        return gripperAxonAnalogRight;
    }

    public AnalogInput getAxonAnalogRight() {
        return axonAnalogRight;
    }

    public AnalogInput getAxonAnalogLeft() {
        return axonAnalogLeft;
    }

    public Motor getLinearSlidesRight1() {
        return linearSlidesRight1;
    }

    public Motor getLinearSlidesRight2() {
        return linearSlidesRight2;
    }


    public Motor getLinearSlidesLeft() {
        return linearSlidesLeft1;
    }

    public Motor getIntakeMotor() {
        return intakeMotor;
    }

    public Motor getRightBackMotor() {
        return rightBackMotor;
    }

    public Motor getLeftBackMotor() {
        return leftBackMotor;
    }

    public Motor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public Motor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public int getOdoReadingLeft() {
        return rightBackMotor.getCurrentPosition();
    }

    public int getOdoReadingPerpendicular() {
        return leftBackMotor.getCurrentPosition();
    }

    public int getOdoReadingRight() {
        return leftFrontMotor.getCurrentPosition();
    }

    public Servo getOdoRetractionLeft() {
        return OdoRetractionLeft;
    }

    public Servo getOdoRetractionRight() {
        return OdoRetractionRight;
    }

    /*public Servo getOdoRetractionMiddle() {
        return OdoRetractionMiddle;
    }*/

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public IMU getImu() {
        return imu;
    }

    public RevBlinkinLedDriver getLeftBlinkin() {
        return leftBlinkin;
    }

    public RevBlinkinLedDriver getRightBlinkin() {
        return rightBlinkin;
    }

    public void clearCache(){
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
    public void updateHeading(boolean share) {
        if (share) {
            imu.resetYaw();
        }
    }
}