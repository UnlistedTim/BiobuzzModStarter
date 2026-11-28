package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

// dist, speed, hood

// 22.047, 1050, 0.31

//25.76, 1060, 0.31

//34.95 , 1090, 0.33

//39.76, 1090, 0.335

// 49, 1120, 0.34

//57.4, 1130,  0.35






















//

//distance, speed, hood

//20.79, 1090, 0.315

//38.23, 1120, 0.335

//45.26, 1130, 0.345


//56.61, 1180, 0.35







//0.105 hood
//0.08 hood
//0.105
//0.145

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

//46


//shooting auto start x:-12.58
//shooting auto start x:-55.79

//shooting auto start x new: -11.65
//shooting auto start Y new: -55.6 distance 35.36 flyvel 1110 ; hood 0.335




//-----------

//r = 27.22
//fly vel 1070 hood 0.32

//-----
// r =59.22
//fly vel 1190 hood 0.345
//

// ------

// r  = 43.955 hood 0.338 ; fly vel  1120
// -----
// r  = 78.38 hood 0.38 flyvel 1280




@TeleOp(name="ModStarterDebug", group="A")
@Config


public class ModStarterDebug extends LinearOpMode  {

    public base rbg ;


    Pose pose;


    private DcMotorEx intakeBack,intakeFront, Flywheel,  leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker, Flipper;

    private CRServo reverseRoller;
    double Tx=0, Ty=0;

    double dist = 0;

   // private CRServo leftRoller, rightRoller;
   private Limelight3A Limelight;

    public static double flyp = 0.012, flyi=0, flyd = 0, flyv = 0.00036, flys = 0.04;

    public static double intakeP = 0.0006, intakeI= 0, intakeD = 0, intakeV = 0.00038, intakeS = 0.05;

    private VoltageSensor voltageSensor;

    public static double intakeFrontPower = 0, intakeBackPower = 0;

    public static int flyVel  = 0;

    public static int intakeVel  = 0;

    public double flyCurrentVel = 0;

    public double intakeCurrentVel = 0;


    public static double hoodPos = 0.38; // 0.28 90 degree up, 0.39 fully in

    public static double blockerPos = 0.27; //0.27 up 0.63 collapse

    public static double flipperPos = 0.4; //0.75 up // 0.4 or 0.35 down

    public static double reverseRollerPower = 0.0;



    public Follower follower;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public double startX = -16.01, startY = -60.89;

    private final PoseFactory p = PoseFactory.degrees();

    public static double targetX = -12.7, targetY = -16;

    private Pose startPose = p.of(-16.01,-62.3,-90);

    private Pose targetPose = p.of(-12.7,-16,-90);

    public double targetHeading = 0;
    LLResult result;

    public static double leftRollerPower = 0, rightRollerPower = 0;

    public static double targetHeadingPIDTest = 0;
    public static double driveP = 0.01, driveI = 0, driveD = 0.0008, driveKs = 0.09;






    public enum State {

        IDLE,
        INTAKE,
        OUTTAKE,
        MANUALOUTTAKE,

        PARK;
    }

    private enum ShootState {

        START,PRE_SHOOT, UNLOCK,SHOOT, DONE
    }
    ShootState shootState = ShootState.START;

 //    State state = State.DEBUG;
 State state = State.IDLE;

    // --- Timers ---
    private ElapsedTime deltaT = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();

    private PIDController drivePID = new PIDController(0,0,0);



    @SuppressLint("SuspiciousIndentation")
    @Override

    public void runOpMode() {

//        rbg= new base();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        initalize();
        drivePID.setTolerance(2);


//        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));
//        pose = Pinpoint.getPosition();
        waitForStart();
        Limelight.pipelineSwitch(1);
        Limelight.start();



//        afterstart();

//        looptimer.reset();
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //Bulk reading for faster loop times

//        Intake.setVelocity(rbg.intakeVel);
        telemetry.clearAll();
        while (opModeIsActive()) { //Main While loop

            follower.update();
            pose=follower.pose();

            dist = pose.distance(targetPose)-5.4;;

            targetPose = new Pose(targetX,targetY, 90);

            targetHeading = Math.toDegrees(Math.atan2(targetPose.y() - pose.y(), targetPose.x() - pose.x()));









            flyCurrentVel = Flywheel.getVelocity();
            Flywheel.setPower(flyPIDF(flyCurrentVel, flyVel));

            intakeCurrentVel = intakeBack.getVelocity();
            intakeBack.setPower(intakePIDF(intakeCurrentVel, intakeVel));



            reverseRoller.setPower(reverseRollerPower);



            Hood.setPosition(hoodPos);

            Flipper.setPosition(flipperPos);

            Blocker.setPosition(blockerPos);

            intakeFront.setPower(intakeFrontPower);

//            intakeBack.setPower(intakeBackPower);

//            leftRoller.setPower(leftRollerPower);
//            rightRoller.setPower(rightRollerPower);







//            dashboardTelemetry.addData("Flywhel vel", flyCurrentVel);
//            dashboardTelemetry.addData("Flywhel current", Flywheel.getCurrent(CurrentUnit.MILLIAMPS));
//
//            dashboardTelemetry.addData("Current X", pose.x());
//            dashboardTelemetry.addData("Current Y", pose.y());
//            dashboardTelemetry.addData("Current theta", Math.toDegrees(pose.heading()));
//            dashboardTelemetry.addData("distance", dist);

//            mecanumRobotCentricDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

//            if (gamepad1.right_bumper){
//                autoAlign();
//            }
            drivePID.setPID(driveP,driveI,driveD);
            result=Limelight.getLatestResult();
            if(result.isValid())  {
                Tx=result.getTx();
                Ty=result.getTy();
            }
            else {
                Tx=1000;
                Ty=1000;
            }


            telemetry.addData("Limelight see?", result.isValid());
            telemetry.addData("Tx", Tx);
            telemetry.addData("Ty", Ty);
            telemetry.update();


            double turn =  mecanumPID(Math.toDegrees(pose.heading()),targetHeadingPIDTest);
            //
//
//            if (!drivePID.atSetPoint()){
//                leftFront.setPower(turn);
//                leftBack.setPower(turn);
//                rightFront.setPower(-turn);
//                rightBack.setPower(-turn);
//
//            }
//            else{
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightFront.setPower(0);
//                rightBack.setPower(0);
//            }



            dashboardTelemetry.update();






        }
    }

    public void initalize() {

        Hw_init();

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
//        rbg.init();
//        getAutoVars();

        // buildPaths();


        // telemetry.clear();
    }









    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");





        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Flipper = hardwareMap.get(Servo.class,"Flipper");

        reverseRoller = hardwareMap.get(CRServo.class,"reverseRoller");

//        leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
//        rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

//        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        intakeFront = hardwareMap.get(DcMotorEx.class,"intakeFront");
        intakeBack = hardwareMap.get(DcMotorEx.class,"intakeBack");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


      //  turretPosition=hardwareMap.get(DcMotorEx.class, "leftFront");


//        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
//        configurePinpoint();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Flipper.setDirection(Servo.Direction.REVERSE);


        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reverseRoller.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");





    }

    public double flyPIDF(double currentVel, double targetVel){
        double velError = targetVel-currentVel;
        double pid = flyp * velError;
        double ff = flyv * targetVel + Math.signum(velError) * flys;
        dashboardTelemetry.addData("Total power", pid+ff);
        dashboardTelemetry.addData("PID", pid);
        dashboardTelemetry.addData("FF", ff);

        return pid + ff;
    }

    public double intakePIDF(double currentVel, double targetVel){

        if (Math.abs(targetVel) < 20) return 0.0;
        double velError = targetVel-currentVel;
        double pid = intakeP * velError;
        double ff = intakeV * targetVel + Math.signum(velError) * intakeS;
        dashboardTelemetry.addData("Total power", pid+ff);
        dashboardTelemetry.addData("PID", pid);
        dashboardTelemetry.addData("FF", ff);
        dashboardTelemetry.addData("current Vel intake", currentVel);
        dashboardTelemetry.addData("target Vel intake", targetVel);



        dashboardTelemetry.addData("Intake back current", intakeBack.getCurrent(CurrentUnit.MILLIAMPS));

        return pid + ff;
    }




    public boolean stoptimers(double period, int i) {

//        if (period == 0) {
//            stoptime[i] = runtime.milliseconds();
//            return false;
//        }
//        return runtime.milliseconds() - stoptime[i] > period;

        return true;
    }

//    public void configurePinpoint(){
//        /*
//         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
//         *
//         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
//         *  Left of the center is a positive number, right of center is a negative number.
//         *
//         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
//         *  Forward of center is a positive number, backwards is a negative number.
//         */
//        Pinpoint.setOffsets(-3, -4.8, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
//
//        /*
//         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
//         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
//         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
//         * number of ticks per unit of your odometry pod.  For example:
//         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
//         */
//        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//
//        /*
//         * Set the direction that each of the two odometry pods count. The X (forward) pod should
//         * increase when you move the robot forward. And the Y (strafe) pod should increase when
//         * you move the robot to the left.
//         */
//        Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//        /*
//         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
//         * The IMU will automatically calibrate when first powered on, but recalibrating before running
//         * the robot is a good idea to ensure that the calibration is "good".
//         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
//         * This is recommended before you run your autonomous, as a bad initial calibration can cause
//         * an incorrect starting value for x, y, and heading.
//         */
//        Pinpoint.resetPosAndIMU();
//    }

    public void mecanumRobotCentricDrive(double y, double x, double rx){


        // Rotate the movement direction counter to the bot's rotation
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        //   telemetry.addData("Angle", pose.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
//        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));



        //  telemetry.update();
    }

    public double mecanumPID(double currentHeading, double targetHeading){
        double error = ((targetHeading - currentHeading + 540) % 360) - 180;
        dashboardTelemetry.addData("error",error);
        return drivePID.calculate(error) - Math.signum(error) * driveKs;



    }



    public void autoAlign(){
        double turn = mecanumPID(Math.toDegrees(pose.heading()),targetHeading );
        if (!drivePID.atSetPoint()){
            leftFront.setPower(-turn);
            leftBack.setPower(-turn);
            rightFront.setPower(turn);
            rightBack.setPower(turn);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }




    }












}




















