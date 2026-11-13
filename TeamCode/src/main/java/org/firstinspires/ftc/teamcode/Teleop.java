package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.bluetooth.le.ScanSettings;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import java.util.List;


@TeleOp(name="ModStarterTeleop", group="A")
@Config


public class Teleop extends LinearOpMode  {

    Pose pose =  new Pose(0,0,0);
    public base rbg ;

    public boolean fieldCentric = true;

    private DcMotorEx intakeBack,intakeFront, flyWheel,  leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker, Flipper;




    private VoltageSensor voltageSensor;

    public double colorFactor = 1;


    double dist = 0;
    //public final double flyp = 0.012, flyi=0, flyd = 0, flyv = 0.00036, flys = 0.04;



    public final double hoodCollapse = 0.38, hoodExtend = 0.28, hoodShoot = 0.35;


//    public final double blockClose = 0.63, blockOpen = 0.27;



//    public static final double  startX1 = -16.01 , startY1 = -55.49;
  public double  startX = 0 , startY = 0,startHeading=0;


    private final PoseFactory p = PoseFactory.degrees();

//    public double targetX = -23+11, targetY = -70.6+55;



    public double targetHeading = 0,currentHeading=0;

    public  Pose startPose; //red start

  private  Pose targetPose = new Pose(0,0,0) ;
//    private  Pose targetPoseredl = p.of(targetXredl,targetYredl,-90);
//    private  Pose targetPoseredh = p.of(targetXredh,targetYredh,-90);

    public boolean colorRed=true;




//    public double Xred = 58, Xb = Xred + 24;
//
//    public double Yback = 54, Yfront = Yback + 2*18;
    //public double posex,posey;



  //  public static double blockerPos = 0.27; //0.27 up 0.63 collapse

    //public final int shootVel = 1180;
    public Follower follower;

    public double flyCurrentVel = 0, targetFlyVel = 0;
    public boolean outtakePhase=false,timer=false,aligNing=false,driVeable=true, intakeTimer = false, outtakeTimer = false;
    private boolean flipperFlag=false,recevieinfo=false,intakejam=false,shooting=false,reversing=false;


    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public final int intake = 0, outtake = 1,flip=2,align=3,reverse=4;
    public int intakestep=0;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runtime = new ElapsedTime();



    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    public enum State {

        IDLE,
        INTAKE,
        ALIGN,
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
        rbg= new base();
        initalize();
        getAutoVars();

        // buildPaths();

        if (colorRed) telemetry.addLine("Red  Selected");
        else telemetry.addLine("Blue  Selected");
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");
        telemetry.addLine("Gunner Right Bumper Confrim afters election");
        telemetry.addLine("*******************************************");
        telemetry.update();
        stoptimers(0,flip);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                colorRed = false;
                recevieinfo = true;

            }
            if (gamepad1.circle) {
                colorRed = true;
                recevieinfo = true;

            }
            if (recevieinfo) {

                break;
            }

        }


        if (colorRed) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");

        telemetry.update();
        if (recevieinfo) {
            startPose= rbg.startpos(colorRed);
            colorFactor = -1 + 2 * Boolean.compare(colorRed, false);
            recevieinfo=false;
        }


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

            follower.update();
            pose=follower.pose();


           if(outtakePhase) statusupdate();
           if( outtakeTimer) timeraction();
           if (gamepad2.rightBumperWasPressed()||(intakejam)){
                if(!intakejam) {intakejam=true;}
                if(intakejam()){intakejam=false;}

            }




            switch (state){
                case IDLE:

                    flyWheel.setPower(0.4);
                    intakeFront.setPower(0.7);
                    intakeBack.setPower(0.7);
//                    leftRoller.setPower(0.9);
//                    rightRoller.setPower(0.9);
                    intakestep=0;
                    state = State.INTAKE;
                    break;


                case INTAKE:
                    if(gamepad2.leftBumperWasPressed()&&stoptimers(1000,flip)){

                        stoptimers(0,flip);
                        flipperFlag=!flipperFlag;
                        if(flipperFlag)
                            Flipper.setPosition(rbg.flipperExtend);
                        else  Flipper.setPosition(rbg.flipperCollapse);
                    }


                    if(gamepad1.rightBumperWasPressed()) {
                        state = State.ALIGN;
                        driVeable = false;
                        outtakePhase=true;
                        stoptimers(0,align);
                    }

                    if(gamepad1.left_bumper){
                        intakeBack.setPower(0.7);

                    }

                    break;


                case ALIGN:


                    if(align()||stoptimers(2000,align)) {

                           state=State.OUTTAKE;
                           intakeBack.setPower(-0.7);
                      //  stoptimers(0,outtake);

                        }



                    break;



                case OUTTAKE:

                    if ( !driVeable && (Math.abs(flyCurrentVel - targetFlyVel) <40||stoptimers(2000,outtake)))
                    {
                      //  intakeBack.setPower(0.5);
                        Blocker.setPosition(rbg.blockOpen);
                        stoptimers(0,outtake);
                        shooting = true;//
                        intakeBack.setPower(0.7);
                    }
                     if(shooting) {
                         if(gamepad1.left_bumper&&!reversing) {
                             stoptimers(0,reverse);
                             reversing=true;
                             intakeBack.setPower(-0.8);
                         }
                         if(reversing&&stoptimers(600,reverse)) {
                             intakeBack.setPower(0.7);
                             reversing=false;
                         }



                     }

                    if(shooting&&Math.abs(gamepad1.right_stick_y)>0.18) {
                        outtakePhase=false;
                        state=State.IDLE;
                        driVeable=true;
                        shooting=false;
                        Blocker.setPosition(rbg.blockClose);
                    }

                    break;

            }

            if (gamepad1.crossWasPressed()) {
                fieldCentric = !fieldCentric;
                gamepad1.rumble(1,1,500);
            }

//            telemetry.addData("Robot Heading", Math.toDegrees(pose.heading()));
//            telemetry.addData("Target Heading", targetHeading);
//            telemetry.addData("Current x", pose.x());
//            telemetry.addData("Current y", pose.y());
//            telemetry.addData("Target pose x", targetPose.x());
//            telemetry.addData("Target pose y", targetPose.y());
//            telemetry.addData("Robot state", state);
//            telemetry.update();


//            dashboardTelemetry.addData("Current X", pose.x());
//            dashboardTelemetry.addData("Current Y", pose.y());
//            dashboardTelemetry.addData("Current theta", pose.heading());
//            dashboardTelemetry.addData("distance", pose.distance(targetPose));
//            dashboardTelemetry.update();

            if(driVeable){
                if (fieldCentric) mecanumRobotDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                else mecanumRobotCentricDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }




        }
    }

    // C =PI/4+arctan (H/r);
    // Vmin=squroot (g(H+squroot(H*H+r*r)

    //

    public void statusupdate(){ //TODO


        flyCurrentVel = flyWheel.getVelocity();
       // double offset=0;
        currentHeading=Math.toDegrees(pose.heading());
        if(pose.y()>0) {
           if(colorRed) targetPose = rbg.targetPoseredh;
           else targetPose = rbg.targetPoseblueh;
        }
         else {
           //  offset=180;
            if(colorRed) targetPose = rbg.targetPoseredl;
            else targetPose = rbg.targetPosebluel;
        }
        targetHeading = Math.toDegrees(Math.atan2( pose.y()-targetPose.y() ,   pose.x()-targetPose.x()));
         if(targetHeading<0) targetHeading=targetHeading+360;
        dist = pose.distance(targetPose)-5.4;
        Hood.setPosition(rbg.flyHood(dist));
        targetFlyVel = rbg.flySpeed(dist);
        flyWheel.setPower(rbg.flyPIDF(flyCurrentVel,targetFlyVel));



    }

    public boolean timeraction()

    {
//       if(intakeTimer ){
//           if (stoptimers(1000,intake)) {
//               intakeFront.setPower(0);
//               intakeTimer = false;
//               return true;
//           }
//       }

//       if (outtakeTimer){
//           if (stoptimers(1000,outtake)) {
//               Blocker.setPosition(rbg.blockClose);
//               outtakeTimer = false;
//               return true;
//           }
//
//       }


       return false;

    }




    public boolean  intakejam()

   {
       switch (intakestep) {

           case 0:
               intakeBack.setPower(-0.7);
               intakeBack.setPower(-0.9);
               stoptimers(0,intake);

               intakestep=5;


               break;


           case 1:
               if(stoptimers(1000,intake)) {
                   intakeBack.setPower(0.7);
                  intakeFront.setPower(0.9);
                   intakestep=0;
                   intakestep=5;
                   return true;
               }




               break;




               }


       return false;





   }


    public boolean align(){

        double turnp=rbg.flooralign(currentHeading,targetHeading);
        leftFront.setPower(turnp);
        leftBack.setPower(turnp);
        rightFront.setPower(-turnp);
        rightBack.setPower(-turnp);
        return turnp==0;
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


        try {
            startHeading = (double) blackboard.get("H");
        }

        catch (NullPointerException e){
            startHeading =0;
        }

        try{
            colorRed = (boolean) blackboard.get("COLOR");
        }

        catch (NullPointerException e){
            colorRed = true;
        }


        if (startX != 0){
             startPose = p.of(startX,startY,startHeading);
        }
        else{
            startPose = rbg.startpos(true);
        }

//        try{
//            Tgap = (int) blackboard.get("T");
//        }
//        catch (NullPointerException e){
//            Tgap= 0;
//        }


    }










    public void initalize() {

        Hw_init();


//        follower.setPose(startPose);




//        rbg.init();
//        getAutoVars();

        // buildPaths();


        // telemetry.clear();
    }

    public void afterstart() {

        pose = follower.pose();
        runtime.reset();
        Blocker.setPosition(rbg.blockClose);
        Hood.setPosition(hoodShoot);
        Flipper.setPosition(rbg.flipperCollapse);
        follower.setPose(startPose);

    }









    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        flyWheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        follower = Constants.create(hardwareMap);

//        leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
//        rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

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
        rbg. LUT_init();
    }

//    public void LUT_init(){
//
//        Flylut.add(0 , 1000); Hoodlut.add(0 , 0.3);
//        Flylut.add(20.79 , 1090); Hoodlut.add(20.79 , 0.315);
//        Flylut.add(38.23 , 1120); Hoodlut.add(38.23 , 0.335);
//        Flylut.add(45.26,1130); Hoodlut.add(45.26,0.345);
//        Flylut.add(56.61,1180); Hoodlut.add(56.61,0.35);
//        Flylut.add(200,1250); Hoodlut.add(200,0.37);
//
//        Flylut.createLUT();
//        Hoodlut.createLUT();
//
//    }






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

    public void mecanumRobotDrive(double y, double x, double rx){

        y*= colorFactor;
        x*=colorFactor;


//        y*= colorFactor;
//        x*=colorFactor;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double botHeading = pose.heading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

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




















