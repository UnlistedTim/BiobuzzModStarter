package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;


import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;


import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import java.util.List;


@TeleOp(name="ModStarterTeleop", group="A")
@Config


public class ModStarterTeleop extends LinearOpMode  {

    Pose pose;

    private DcMotorEx intakeBack,intakeFront, flyWheel,  leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker,Flower, Flipper;

    private CRServo leftRoller, rightRoller;

    private VoltageSensor voltageSensor;

    private PIDController drivePID = new PIDController(0.015,0,0.0008);
 //   private PIDController drivePID = new PIDController(0.01,0,0.0008);

    public final double driveKs = 0.1;

    boolean rotating = false;


    InterpLUT Flylut = new InterpLUT();

    InterpLUT Hoodlut = new InterpLUT();

    double dist = 0;


    public final double flyp = 0.012, flyi=0, flyd = 0, flyv = 0.00036, flys = 0.04;



    public final double hoodCollapse = 0.38, hoodExtend = 0.28, hoodShoot = 0.35;

    public final double flipperExtend = 0.38, flipperCollapse = 0.75;

    public final double blockClose = 0.63, blockOpen = 0.27;

    public boolean shooting = false, canShoot = false;

    public double startX = -16.01 , startY = -55.49;

    private final PoseFactory p = PoseFactory.degrees();

    public double targetX = -23+11, targetY = -70.6+55;
    public double targetXredl = targetX, targetYredl = targetY;
    public double targetXredh = targetX, targetYredh = -targetY;
    public double targetXbluel = -targetX, targetYbluel = targetY;
    public double targetXblueh = -targetX, targetYblueh = -targetY;


    public double targetHeading = 0,currentHeading=0;

    private final Pose startPose = p.of(startX,startY,-90);

    private  Pose targetPose = p.of(targetX,targetY,-90);
    private  Pose targetPoseredl = p.of(targetXredl,targetYredl,-90);
    private  Pose targetPoseredh = p.of(targetXredh,targetYredh,-90);

    public boolean colorRed=true;




    public double Xred = 58, Xb = Xred + 24;

    public double Yback = 54, Yfront = Yback + 2*18;
    public double posex,posey;



    public static double blockerPos = 0.27; //0.27 up 0.63 collapse

    public final int shootVel = 1180;
    public Follower follower;

    public double flyCurrentVel = 0, targetFlyVel = 0;
    public boolean outtakePhase=false,timer=false,aligNing=false,driVeable=true, intakeTimer = false, outtakeTimer = false;








    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public final int intake = 0, outtake = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runtime = new ElapsedTime();



    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    public enum State {

        IDLE,
        INTAKE,
        ALIGN,
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
//    private ElapsedTime outtakeTimer = new ElapsedTime();



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


           if(outtakePhase) statusupdate();
           if(intakeTimer || outtakeTimer) timeraction();



            switch (state){
                case IDLE:

                    flyWheel.setPower(0.4);
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    leftRoller.setPower(0.9);
                    rightRoller.setPower(0.9);
                    state = State.INTAKE;
                    break;


                case INTAKE:
                    if (gamepad2.rightBumperWasPressed()){
                        intakeFront.setPower(-0.9);
                        leftRoller.setPower(0);
                        rightRoller.setPower(0);
                        intakeTimer = true;
                        stoptimers(0,intake);
                        outtakePhase=true;
                        state=State.ALIGN;
                    }

                    break;


                case ALIGN:

                    if(gamepad1.rightBumperWasPressed()||aligNing) {
                         aligNing=true;
                         driVeable=false;
                         if(flooralign()) {
                           aligNing=false;
                           state=State.OUTTAKE;

                        }


                    }

                    break;



                case OUTTAKE:

                    if ( !driVeable && Math.abs(flyCurrentVel - targetFlyVel) <40){
                      //  intakeBack.setPower(0.5);
                        Blocker.setPosition(blockOpen);
                        stoptimers(0,outtake);
                        outtakeTimer = true;//
                        driVeable=true;

                    }
                    if(driVeable&&!outtakeTimer) {
                        outtakePhase=false;
                        state=State.IDLE;
                    }

                    break;

            }

            telemetry.addData("State", state);
            telemetry.addData("Intake timer", intakeTimer);
            telemetry.addData("Outtake timer", outtakeTimer);
            telemetry.update();


//            dashboardTelemetry.addData("Current X", pose.x());
//            dashboardTelemetry.addData("Current Y", pose.y());
//            dashboardTelemetry.addData("Current theta", pose.heading());
//            dashboardTelemetry.addData("distance", pose.distance(targetPose));
//            dashboardTelemetry.update();

            if(driVeable)mecanumRobotCentricDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);




        }
    }

    // C =PI/4+arctan (H/r);
    // Vmin=squroot (g(H+squroot(H*H+r*r)

    //

    public void statusupdate(){ //TODO
        follower.update();
        pose=follower.pose();
        currentHeading=Math.toDegrees(pose.heading());
        if(pose.y()>0) targetPose=targetPoseredh; else targetPose=targetPoseredl;
        targetHeading = Math.toDegrees(Math.atan2(targetPose.y() - pose.y(), targetPose.x() - pose.x()));
        dist = pose.distance(targetPose);
        targetFlyVel = flySpeed(dist);
        flyCurrentVel = flyWheel.getVelocity();
        Hood.setPosition(flyHood(dist));

        flyWheel.setPower(flyPIDF(flyCurrentVel,targetFlyVel ));



    }

    public boolean timeraction()

    {
       if(intakeTimer ){
           if (stoptimers(1000,intake)) {
               intakeFront.setPower(0);
               intakeTimer = false;
               return true;
           }
       }

       if (outtakeTimer){
           if (stoptimers(1000,outtake)) {
               Blocker.setPosition(blockClose);
               outtakeTimer = false;
               return true;
           }

       }


       return false;

    }

    public boolean flooralign(){

        double offset=180;
       double shootHeading =(currentHeading+offset)%360;
        double turnPower;
        double error=targetHeading-shootHeading ;
        if (Math.abs(error) > 180)
            error =-Math.signum(error) *(Math.abs(error)-360);// change rotation direction


//        dashboardTelemetry.addData("error",error);
       // return drivePID.calculate(error) - Math.signum(error) * driveKs;



       turnPower= drivePID.calculate(error);
//        telemetry.addData("Pid", turnPower);
        turnPower+= Math.signum(turnPower) * driveKs;
//        telemetry.addData("Pid + Feedforward", turnPower);
//        telemetry.update();



//        telemetry.addData("turn Power ", turnPower);
//        telemetry.addData("Error", error);
//        telemetry.update();

        if (Math.abs(error)> 2){
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            return true;

        }

        return false;


    }


    public void getAutoVars(){

//
//        try {
//            startHeading = (double) blackboard.get("Heading");
//        }
//        catch (NullPointerException e){
//            startHeading = 0;
//        }

        try {
            startX = (double) blackboard.get("X");
        }

        catch (NullPointerException e){

            startX = 0;

        }

        try {
            startY = (double) blackboard.get("Y");
        }

        catch (NullPointerException e){
            startY =0;
        }

        try{
            colorRed = (boolean) blackboard.get("COLOR");
        }
        catch (NullPointerException e){
            colorRed = true;
        }

//        try{
//            Tgap = (int) blackboard.get("T");
//        }
//        catch (NullPointerException e){
//            Tgap= 0;
//        }


    }

    public void autoAlign(){
        double turn = mecanumPID(Math.toDegrees(pose.heading())+180,targetHeading );

//        telemetry.addData("Pose angle", Math.toDegrees(pose.heading()));
//        telemetry.addData("Target angle", targetHeading);
//        telemetry.addData("Error gap",  targetHeading-Math.toDegrees(pose.heading()));
//        telemetry.update();
        if (Math.abs(targetHeading-(Math.toDegrees(pose.heading()) - 180)) > 1 ){
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
            rotating = false;
            canShoot = true;
        }




    }

    public double mecanumPID(double currentHeading, double targetHeading){
        currentHeading +=180;
        double error;
        if (currentHeading - targetHeading < 180){
            error = targetHeading-currentHeading;
        } else{
            error = currentHeading-targetHeading;
        }


        return drivePID.calculate(error) - Math.signum(error) * driveKs;



    }


    public void initalize() {
//        follower = Constants.createFollower(hardwareMap);
        Hw_init();
        LUT_init();
        follower.setPose(startPose);
//        follower.setPose(startPose);




//        rbg.init();
//        getAutoVars();

        // buildPaths();


        // telemetry.clear();
    }

    public void afterstart() {

        pose = follower.pose();
        // deltaT.reset();
        runtime.reset();

        Blocker.setPosition(blockClose);
        Hood.setPosition(hoodShoot);
        Flipper.setPosition(flipperCollapse);


    }









    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        flyWheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        follower = Constants.create(hardwareMap);

        leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Flipper = hardwareMap.get(Servo.class, "Flipper");
        //Flower = hardwareMap.get(Servo.class, "Flower");

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

        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);


        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void LUT_init(){

        Flylut.add(0 , 1000); Hoodlut.add(0 , 0.3);
        Flylut.add(20.79 , 1090); Hoodlut.add(20.79 , 0.315);
        Flylut.add(38.23 , 1120); Hoodlut.add(38.23 , 0.335);
        Flylut.add(45.26,1130); Hoodlut.add(45.26,0.345);
        Flylut.add(56.61,1180); Hoodlut.add(56.61,0.35);
        Flylut.add(200,1250); Hoodlut.add(200,0.37);

        Flylut.createLUT();
        Hoodlut.createLUT();

    }

    public double flyHood(double dist){
        if (dist < 56.6 && dist > 20.7) {

            return Hoodlut.get(dist);

//            Tx_offset=0;

        }
        else return 0.34;//1500
    }

    public double flySpeed(double dist){
        if (dist < 56.6 && dist > 20.7) {

            return Flylut.get(dist);// Tx offset

//            Tx_offset=0;

        }
        else return 1130;


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

    public double flyPIDF(double currentVel, double targetVel){
        double velError = targetVel-currentVel;
        double pid = flyp * velError;
        double ff = flyv * targetVel + Math.signum(velError) * flys;

//        telemetry.addData("flypid power" , pid + ff);
//        dashboardTelemetry.addData("Total power", pid+ff);
//        dashboardTelemetry.addData("PID", pid);
//        dashboardTelemetry.addData("FF", ff);

        return pid + ff;
    }












}




















