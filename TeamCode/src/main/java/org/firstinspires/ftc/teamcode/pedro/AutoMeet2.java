package org.firstinspires.ftc.teamcode.pedro;

import static android.os.SystemClock.sleep;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.base;


@Autonomous
public class AutoMeet2 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Pose pose,targetPose;
    private double currentH,targetH,flyCurrentVel,dist = 35.36 ,targetflyvel;


    private int pathState=0;



    ElapsedTime autotime = new ElapsedTime();

//    private final Pose scorePose = poseFactory.of(48, 48, 90);

    private DcMotorEx intakeBack,intakeFront, flyWheel;

    private Servo Hood, Blocker, Flipper;

//    private CRServo leftRoller, rightRoller;

    public base rbga=new base();
    private final Pose parkRed1 = poseFactory.of(-50, 20, -90);
    private final Pose parkPoseblue1 = poseFactory.of(53, -30, 90);

    private final Pose shootRedL = poseFactory.of(-11.65, -55.6, -90);
    private final Pose shootBlueH = poseFactory.of(11.65, 55.6, 90);
    private final Pose ctlshootRedH = poseFactory.of(-46, 46, 0);

    private final Pose shootRedH = poseFactory.of(-11.65, 55.6, 90);
    private final Pose shootBlueL = poseFactory.of(-11.65, 55.6, 90);
    private final Pose parkPosectlred1 = poseFactory.of(-47, -47, -90);
    private final Pose parkPosectlblue1 = poseFactory.of(47, 47, 90);
    private final Pose flowerRedL = poseFactory.of(-58, -23, -180);
    private final Pose flowerRedH = poseFactory.of(-23, -58, 90);
    private final Pose cornerRed= poseFactory.of(-57, -47, -90);




    //  private double startX = -16.01 , startY = -60.89,startHeading=-90;

    public boolean recevieinfo=false,colorRed=true,configured=false,update=true;

   // public  Pose startPose = poseFactory.of(startX,startY,-90);
   public  Pose startPose ;
    // Path methods
//    private Path startToScore() {
//        return line(startPose, scorePose).constant(startPose);
//    }



    private Path parkred1(){
        return Paths.curve(shootRedL, parkPosectlred1,parkRed1).constant(Math.toRadians(-90));
       // return line()
    }
    private Path parkblue1(){
        return Paths.curve(shootBlueH, parkPosectlblue1,parkPoseblue1).constant(Math.toRadians(90));
        // return line()
    }

    private Path pathstartShootLred(){
        return Paths.line(startPose, shootRedL).constant(Math.toRadians(-90));
        // return line()
    }
    private Path pathstarshootHblue(){
        return Paths.line(startPose, shootBlueH).constant(Math.toRadians(90));
        // return line()
    }
    private Path
    pathshootLflowerLred(){
        return Paths.line(shootRedL, flowerRedL).linear(shootRedL,flowerRedL);
        // return line()
    }
    private Path pathflowerLshootHred(){
        return Paths.curve(flowerRedL, ctlshootRedH,shootRedH).linear(flowerRedL,shootRedH);
        // return line()
    }
    private Path pathshootHflowerHred(){
        return Paths.line(shootRedH, flowerRedH).constant(Math.toRadians(90));
        // return line()
    }
    private Path pathflowerHshootHred(){
        return Paths.line(flowerRedH,shootRedH).constant(Math.toRadians(90));
        // return line()
    }

    private Path pathshootHcornerred(){
        return Paths.curve(shootRedH, ctlshootRedH,cornerRed).linear(shootRedH,cornerRed);
        // return line()
    }
    private Path pathcornershootHred(){
        return Paths.curve(cornerRed, ctlshootRedH,shootRedH).linear(cornerRed,ctlshootRedH);
        // return line()
    }

    private Path pathshootHparkred(){
        return Paths.line(shootRedH,parkRed1).constant(Math.toRadians(90));
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
        startPose=rbga.startpos(colorRed);
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
        Pathupdate1();

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



    void Pathupdate1() {

        switch (pathState) {

            case 0:
                

                if(colorRed)follower.follow(pathstartShootLred());
           //     else follower.follow(firstShootblue1());
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
                  //  flyWheel.setPower(0);
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    Blocker.setPosition(rbga.blockClose);
                    if(colorRed)   follower.follow(pathshootLflowerLred());

                   // update=false;
                    pathState=7;

                }
                break;


            case 7:
                if(!follower.isBusy()) {
                  //  Blocker.setPosition(rbga.blockClose);
                    Flipper.setPosition(rbga.flipperExtend);
                    follower.manual(0.25,0,0);
                    rbga.timer(0);
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    pathState=9;

                }
                break;

            case 9:
                if(rbga.timer(1000)) {
                    follower.manual(0,0,0);
                    pathState=15;
                }

                break;
            case 15:
                if(rbga.timer(2000))
                {if(colorRed)   follower.follow(pathflowerLshootHred());
                    intakeBack.setPower(0);
                    intakeFront.setPower(0);

                pathState=17;
                rbga.timer(0);
                }
                break;

            case 17:
                if(rbga.timer(400)) {
                    Flipper.setPosition(rbga.flipperCollapse);
                    pathState=20;
                }

                break;

            case 20:
                if(!follower.isBusy()) {
                    pathState=25;
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                }

                break;

            case 25:
                if(shoot()) {pathState=30;rbga.timer(0);}
                break;


            case 30:
                if(rbga.timer(1500)) {

                    follower.follow(pathshootHflowerHred());
                    Flipper.setPosition(rbga.flipperExtend);
                    Blocker.setPosition(rbga.blockClose);
                }

                break;

            case 35:
                if(!follower.isBusy()) {
                    follower.manual(0.25,0,0);

                    rbga.timer(0);

                    pathState=40;
                }

            case 40:
                if(rbga.timer(1000)){

                    follower.manual(0,0,0);
                    pathState=45;

            }
                break;


            case 45:
                if(rbga.timer(2000)) {

                    follower.follow(pathflowerHshootHred());
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    pathState=55;

                }
                break;



            case 55:
                if(!follower.isBusy()) {
                    pathState=60;
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);

                }
                break;
            case 60:
                if(shoot()) {
                    rbga.timer(0);
                    pathState=65;}
                break;
            case 65:
                if(rbga.timer(1500)) {

                    Blocker.setPosition(rbga.blockClose);
                    pathState=70;
                }
                break;
            case 70:
                follower.follow(pathshootHcornerred());
                pathState=75;
                break;
            case 75:
                if(!follower.isBusy()) {
                    follower.manual(0.3,0,0);
                    rbga.timer(0);
                    pathState=80;
                }
                break;

            case 80:
                if(rbga.timer(1000)) {

                    follower.manual(-0.15,0,0);
                    pathState=85;
                }
                break;

            case 85:
                if(rbga.timer(2000)) {


                    pathState=90;
                }
                break;

            case 90:
                follower.follow(pathcornershootHred());
                pathState=95;
                break;

            case 95:
                if(!follower.isBusy()){

                    pathState=100;
                }
                break;
            case 100:
                if(shoot()) {
                    rbga.timer(0);
                    pathState=105;
                }
                break;
            case 105:
                if(rbga.timer(1500)) {

                    Blocker.setPosition(rbga.blockClose);
                    update=false;
                    flyWheel.setPower(0);
                    intakeBack.setPower(0);
                    intakeFront.setPower(0);
                    follower.follow(pathshootHparkred());

                    pathState=110;


                }
                break;

            case 110:
                if(!follower.isBusy()){

                    pathState=200;
                }
                break;

            case 200:
                sleep(500);
                autosave();
                pathState=210;
            case 210:
                break;

        }
    }


    void autosave()

    {
       // pose=follower.pose();
        blackboard.put("COLOR",  colorRed);
        blackboard.put("X",  follower.pose().x());
        blackboard.put("Y",  follower.pose().y());
        blackboard.put("H",  Math.toDegrees(follower.pose().heading()));

    }

     void Hw_init() {

            flyWheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

//            leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
//            rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

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