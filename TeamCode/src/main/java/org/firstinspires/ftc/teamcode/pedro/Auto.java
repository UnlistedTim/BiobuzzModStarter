package org.firstinspires.ftc.teamcode.pedro;
import static android.os.SystemClock.sleep;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.base;


@Autonomous
public class Auto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Pose pose,targetPose;
    private double currentH,targetH,flyCurrentVel,dist = 35.36 ,flyspeedgap=200,targetflyvel;
    public double startX1 = -16.01 , startY1 = -55.49,startHeading1=-90;
//    public double  startXred1=startX1,startYred1=startY1 ,startHeadingred1=-90;
//    public double  startXblue1=-startX1,startYbue1=-startY1 ,startHeadingblue1=90;
    private double startX = -16.01 , startY = -60.89,startHeading=-90;
    private int pathState=0;

    public  Pose startPose = poseFactory.of(startX,startY,-90);
    private final  double targetX = -23+11, targetY = -70.6+55;
    ElapsedTime autotime = new ElapsedTime();
//    public double targetXredl = targetX, targetYredl = targetY;
//    public double targetXredh = targetX, targetYredh = -targetY;
//    public double targetXbluel = -targetX, targetYbluel = targetY;
//    public double targetXblueh = -targetX, targetYblueh = -targetY;

    // Poses
   // private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPosered1 = poseFactory.of(-53, 30, -90);

    private final Pose startShootRed1 = poseFactory.of(-11.65, -55.6, -90);
    private final Pose parkPosectlred1 = poseFactory.of(-47, -47, -90);
    private DcMotorEx intakeBack,intakeFront, flyWheel;

    private Servo Hood, Blocker, Flipper;

    private CRServo leftRoller, rightRoller;

    public base rbga=new base();



   // private double targetX = -23+11, targetY = -70.6+55;
    public boolean recevieinfo=false,colorRed=true,configured=false,update=true;

    // Path methods
    private Path startToScore() {
        return line(startPose, scorePose).constant(startPose);
    }



    private Path parkred1(){
        return Paths.curve(startPose, parkPosectlred1,parkPosered1).constant(Math.toRadians(-90));
       // return line()
    }

    private Path firstShootred1(){
        return Paths.line(startPose, startShootRed1).constant(Math.toRadians(-90));
        // return line()
    }

//    private Path parkblue1(){
//        return line(scorePose, parkPose).linear(scorePose, parkPose);
//    }


//    private Command autoRoutine() {
//        return sequential(
//                follow(follower, startToScore()),
//                // Add mechanism commands here.
//                follow(follower, parkred1())
//        );
//    }

    @Override
    public void init() {
     //  Scheduler.reset();
        follower = Constants.create(hardwareMap);
        Hw_init();




        // telemetry.addLine("Turn the camera to the shooting target");
        telemetry.addLine("Driver Cross X        ------->  Blue ");
        telemetry.addLine("Driver Circle O       ------->  Red ");
//        telemetry.addLine("Gunner Triangle       ------->  3rd Row Auto");
//        telemetry.addLine("Gunner Square         ------->  Non-3rd Row Auto");
        telemetry.addLine("Gunner right bumper   ------->  Confirm");


        telemetry.update();

    }

    @Override
    public void init_loop() {

//        if(!cameraready&&Limelight.getLatestResult().isValid()){
//            cameraready=true;
//            recevieinfo = true;
//
//        }

//        if(gamepad2.triangle) {
//            recevieinfo = true;
//            row3=true;
//        }
//        if(gamepad2.square) {
//            recevieinfo = true;
//            row3=false;
//        }


        if (gamepad1.crossWasPressed()) {
            colorRed = false;
            recevieinfo = true;

        }



        if (gamepad1.circleWasPressed()) {
            colorRed = true;
            recevieinfo = true;


        }
        if(recevieinfo)

        {
//            telemetry.addLine("Driver Cross X       ------->  Blue ");
//            telemetry.addLine("Driver CircleO       ------->  Red ");
////            telemetry.addLine("Gunner Triangle      ------->  3rd Row Auto");
////            telemetry.addLine("Gunner Square        ------->  Non-3rd Row Auto");
//            telemetry.addLine("Gunner right bumper   -------> Confirm");
//            telemetry.addLine(".......................................");
//            telemetry.addLine("");

//            if(cameraready)  telemetry.addLine("Camera Good!");
//            else  telemetry.addLine("Camera has issue! Pls check or restart it!!!!");
            if(colorRed)    telemetry.addLine("Red");
            else   telemetry.addLine("Blue");
//            if(row3)   telemetry.addLine("Row3 Path");
//            else  telemetry.addLine("Non-Row3 Path");
            telemetry.addLine("Gunner right bumper ------->  confirm");
            recevieinfo=false;
            telemetry.update();

        }

        if(gamepad2.rightBumperWasPressed() && !configured){

            configured=true;
           startPose=rbga.startpos(colorRed);

            //startPose=poseFactory.of(startX,startY,startHeading);

               // Limelight.pipelineSwitch(2);//6 for red
//                rbga.targetGoalY=rbga.targetGoalY-rbga.rfyoffset;
//                rbga.targetGoalX=rbga.targetGoalX-rbga.rfxoffset;




//            }
//            else {
////                autoTurretOffset = 4;
////                Limelight.pipelineSwitch(3);//7 for blue
////                rbga.targetGoalY=rbga.targetGoalY-rbga.bfyoffset;
////                rbga.targetGoalX=0-rbga.bfxoffset;
//
//
//
//            }

           // Limelight.start();

//            blackboard.put("COLOR",  red);
//            telemetry.addLine("Configuraiton confirmed!No change anymore! ");
//            if(cameraready)  telemetry.addLine("Camera Good!");
//            else  telemetry.addLine("Camera has issue! Pls check or restart it!!!!");
//            if(red)    telemetry.addLine("Red");
//            else   telemetry.addLine("Blue");
//            if(row3)   telemetry.addLine("Row3 Path");
//            else  telemetry.addLine("Non-Row3 Path");
//            telemetry.update();



        }


    }

    @Override
    public void start() {
        autotime.reset();
        rbga.basetime.reset();
        follower.setPose(startPose);
        pose = follower.pose();
//        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
       if(update) statusup();
        redPathupdate1();

        //Scheduler.execute();

        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    boolean shoot(){
       if(Math.abs(flyCurrentVel - targetflyvel)<40) {
           Blocker.setPosition(rbga.blockOpen);
           return true;
       }
        return false;
    }
    void statusup()

    {
        pose=follower.pose();

        flyCurrentVel = flyWheel.getVelocity();
        double offset=0;

        currentH=Math.toDegrees(pose.heading());
        if(pose.y()>0) {
            if(colorRed) targetPose = rbga.targetPoseredh;
            else targetPose = rbga.targetPoseblueh;
        }
        else {
           // offset=180;
            if(colorRed) targetPose = rbga.targetPoseredl;
            else targetPose = rbga.targetPosebluel;
        }
        if (dist == 0) dist = pose.distance(targetPose);


        targetflyvel=rbga.flySpeed(dist);
        targetH = Math.toDegrees(Math.atan2(pose.y()-targetPose.y() , pose.x()-targetPose.x() ));
        if(targetH<0) targetH=targetH+360;

        Hood.setPosition(rbga.flyHood(dist));
        flyWheel.setPower(rbga.flyPIDF(flyCurrentVel,targetflyvel) );

    }



    void redPathupdate1() {

        switch (pathState) {

            case 0:
                

                follower.follow(firstShootred1());



                intakeBack.setPower(0.7);
                pathState = 1;
                break;

            case 1:
                if(!follower.isBusy()) pathState = 2;
                break;


            case 2:

                if(shoot()) {
                    rbga.timer(0);
                    pathState=3;
                }

                break;
            case 3:
                if(rbga.timer(1500)) {
                    flyWheel.setPower(0);
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    update=false;
                    pathState=5;
                }
                break;

            case 5:

//                follow(follower,parkred1());
//                follower.followpath
                follower.follow(parkred1());
//                if(!follower.isBusy()) pathState = 100;
               // follow(follower,parkred1());
                pathState=7;

                break;
            case 7:
                if(!follower.isBusy()) pathState = 100;
                break;
            case 9:

                break;
            case 11:

                break;
            case 100:
                sleep(500);
                autosave();
                pathState=110;
            case 110:
                break;

        }
    }


    void autosave()

    {
        pose=follower.pose();
        blackboard.put("COLOR",  colorRed);
        blackboard.put("X",  follower.pose().x());
        blackboard.put("Y",  follower.pose().y());
        blackboard.put("H",  Math.toDegrees(follower.pose().heading()));



    }

     void Hw_init() {

            flyWheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

            leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
            rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

            Hood = hardwareMap.get(Servo.class, "Hood");
            Blocker = hardwareMap.get(Servo.class, "Blocker");
            Flipper = hardwareMap.get(Servo.class, "Flipper");



            intakeFront = hardwareMap.get(DcMotorEx.class,"intakeFront");
            intakeBack = hardwareMap.get(DcMotorEx.class,"intakeBack");

         //   voltageSensor = hardwareMap.voltageSensor.iterator().next();


            //  turretPosition=hardwareMap.get(DcMotorEx.class, "leftFront");


//        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
//        configurePinpoint();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo



            intakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

//            leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);


            flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Flipper.setPosition(rbga.flipperCollapse);

            Blocker.setPosition(rbga.blockClose);

            Hood.setPosition(0.335);


            rbga.LUT_init();
        }


}