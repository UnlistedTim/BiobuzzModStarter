package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;


import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import java.util.List;


@TeleOp(name="ModStarterTeleop", group="A")
@Config


public class ModStarterTeleop extends LinearOpMode  {

    Pose pose;

    private DcMotorEx intakeBack,intakeFront, Flywheel,  leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker;

    private VoltageSensor voltageSensor;


    public final double hoodCollapse = 0.38, hoodExtend = 0.28, hoodShoot = 0.35;

    public final double blockClose = 0.63, blockOpen = 0.27;

    public boolean shooting = false, canShoot = false;

    public double startX = -16, startY = -67.3;

    private final PoseFactory p = PoseFactory.degrees();

    public double targetX = -23+11, targetY = -70.6+55;

    private final Pose startPose = p.of(startX,startY,90);

    private final Pose targetPose = p.of(targetX,targetY,90);




    public double Xred = 58, Xb = Xred + 24;

    public double Yback = 54, Yfront = Yback + 2*18;
    public double posex,posey;



    public static double blockerPos = 0.27; //0.27 up 0.63 collapse

    public final int shootVel = 1180;
    public Follower follower;





    public final PIDFCoefficients flyPIDVals = new PIDFCoefficients(10, 0, 0, 13);
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public final int intake = 0, outtake = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runtime = new ElapsedTime();



    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    public enum State {

        IDLE,
        INTAKE,
        TRANS,
        OUTTAKE
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



    @SuppressLint("SuspiciousIndentation")
    @Override

    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        initalize();


//        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));
//        pose = Pinpoint.getPosition();
        waitForStart();

        afterstart();

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //Bulk reading for faster loop times

//        Intake.setVelocity(rbg.intakeVel);
        telemetry.clearAll();
        while (opModeIsActive()) { //Main While loop
            


            switch (state){
                case IDLE:
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.6);
                    state = State.INTAKE;
                    break;


                case INTAKE:
                    if (gamepad1.rightBumperWasPressed()){
                        intakeFront.setPower(-0.9);
                        stoptimers(0,intake);
                        Flywheel.setVelocity(shootVel);
                        state = State.TRANS;


                    }
                    break;


                case TRANS:
                    if (stoptimers(500, intake)){
                        intakeFront.setPower(0);
                        state = State.OUTTAKE;

                    }
                    break;
                case OUTTAKE:
                    if (gamepad1.rightBumperWasPressed()){
                        canShoot = true;

                    }

                    if (canShoot && Math.abs(Flywheel.getVelocity() - shootVel) <=40){
                        intakeBack.setPower(0.6);
                        Blocker.setPosition(blockOpen);
                        shooting = true;
                        canShoot = false;
                        stoptimers(0,outtake);
                    }

                    if (shooting && stoptimers(1000,outtake)){
                        Blocker.setPosition(blockClose);
                        shooting = false;
                        state = State.IDLE;
                    }
                    break;




            }

            statusupdate();

            dashboardTelemetry.addData("Current X", pose.x());
            dashboardTelemetry.addData("Current Y", pose.y());
            dashboardTelemetry.addData("Current theta", pose.heading());
            dashboardTelemetry.addData("distance", pose.distance(targetPose));

            mecanumRobotCentricDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);




        }
    }

    // C =PI/4+arctan (H/r);
    // Vmin=squroot (g(H+squroot(H*H+r*r)

    //

    public void statusupdate(){
    follower.update();
    pose=follower.pose();


    }
    public void initalize() {
//        follower = Constants.createFollower(hardwareMap);
        Hw_init();
        follower.setPose(startPose);


//        rbg.init();
//        getAutoVars();

        // buildPaths();


        // telemetry.clear();
    }

    public void afterstart() {
        // deltaT.reset();
        outtakeTimer.reset();
        runtime.reset();

        Blocker.setPosition(blockClose);
        Hood.setPosition(hoodShoot);

    }









    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        follower = Constants.create(hardwareMap);

        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");

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
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);


        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flyPIDVals);

        Flywheel.setVelocity(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    }


    public boolean stoptimers(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;

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












}




















