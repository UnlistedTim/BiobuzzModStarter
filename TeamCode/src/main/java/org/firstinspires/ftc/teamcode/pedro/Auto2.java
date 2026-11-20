package org.firstinspires.ftc.teamcode.pedro;

import static android.os.SystemClock.sleep;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base;


@Autonomous
public class Auto2 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Pose pose,targetPose;
    private double targetH,flyCurrentVel,dist = 34.21 ,targetflyvel;//35.36


    private int pathState=0;



    ElapsedTime autotime = new ElapsedTime();

//    private final Pose scorePose = poseFactory.of(48, 48, 90);

    private DcMotorEx intakeBack,intakeFront, flyWheel;

    private Servo Hood, Blocker, Flipper;




    public base rbga=new base();
    private final Pose parkred = poseFactory.of(-55, 25, 90);
    private final Pose shootLred = poseFactory.of(-10, -55.6, -90);
    private final Pose shootLtranred = poseFactory.of(-10, -35, -90);
    private final Pose shootHred = poseFactory.of(-22,52, 99);
    private final Pose shootHtranred = poseFactory.of(-10, 30, -90);
    private final Pose flowerLred = poseFactory.of(-51, -23, 180);
    private final Pose flowerLtranred = poseFactory.of(-40, 25, 90);
    private final Pose flowerHred = poseFactory.of(-24, 59, 99);
    private final Pose cornerred= poseFactory.of(-60, -55, -100);


    private final Pose parkblue = poseFactory.of(55, -25, -90);
    private final Pose shootHblue = poseFactory.of(10, 55.6, 90);
    private final Pose shootHtranblue = poseFactory.of(10, 35, 90);
    private final Pose shootLblue = poseFactory.of(22, -52, -81);
    private final Pose shootLtranblue = poseFactory.of(10, -30, 90);
    private final Pose flowerHblue = poseFactory.of(51, 23, 0);
    private final Pose flowerHtranblue = poseFactory.of(40, -25, -90);
    private final Pose flowerLblue = poseFactory.of(24, -59, -81);
    private final Pose cornerblue = poseFactory.of(60, 55, 80);




    //  private double startX = -16.01 , startY = -60.89,startHeading=-90;

    public boolean recevieinfo=false,colorRed=true,configured=false,update=true,complete=false;

   // public  Pose startPose = poseFactory.of(startX,startY,-90);
   public  Pose startPose ;

    Path corrnershootLred= Paths.line(cornerred,shootLtranred).linear(cornerred,shootLtranred);
    Path shootLshootHtransred= Paths.line(shootLtranred,shootHtranred).constant(Math.toRadians(-90));
    Path shootHtransshootHred= Paths.line(shootHtranred,shootHred).linear(shootHtranred,shootHred);


    Path flowerHflowerLtranred= Paths.line(flowerHred,flowerLtranred).linear(flowerHred,flowerLtranred);
    Path flowerLtranflowerLred= Paths.line(flowerLtranred,flowerLred).linear(flowerLtranred,flowerLred);
    Path flowerLflowerLtranred= Paths.line(flowerLred,flowerLtranred).linear(flowerLred,flowerLtranred);
    Path flowerLtranshootHred= Paths.line(flowerLtranred,shootHred).linear(flowerLtranred,shootHred);


    Path shootHflowerLtranred= Paths.line(shootHred,flowerLtranred).linear(shootHred,flowerLtranred);

    Path flowerLtranparkred= Paths.line(flowerLtranred,parkred).constant(Math.toRadians(90));


    private Path pathstartshootLred(){
        return Paths.line(startPose, shootLred).constant(Math.toRadians(-90));

    }

    private Path  pathshootLcornerred(){
        return Paths.line(shootLred,cornerred).linear(shootLred,cornerred);
        // return line()
    }

    private Path pathcornershootHred(){

        return Paths.path(corrnershootLred,shootLshootHtransred,shootHtransshootHred);
    }

    private Path pathflowerHflowerLred(){

        return Paths.path(flowerHflowerLtranred,flowerLtranflowerLred);
    }




    private Path pathflowerLshootHred(){

        return Paths.path(flowerLflowerLtranred,flowerLtranshootHred);
    }

    private Path pathshootHparkred(){

        return Paths.path(shootHflowerLtranred,flowerLtranparkred);
    }
//blue
    Path corrnershootHblue = Paths.line(cornerblue, shootHtranblue).linear(cornerblue, shootHtranblue);
    Path shootHshootLtransblue = Paths.line(shootHtranblue, shootLtranblue).constant(Math.toRadians(90));
    Path shootLtransshootLblue = Paths.line(shootLtranblue, shootLblue).linear(shootLtranblue, shootLblue);


    Path flowerLflowerHtranblue = Paths.line(flowerLblue, flowerHtranblue).linear(flowerLblue, flowerHtranblue);
    Path flowerHtranflowerHblue = Paths.line(flowerHtranblue, flowerHblue).linear(flowerHtranblue, flowerHblue);
    Path flowerHflowerHtranblue = Paths.line(flowerHblue, flowerHtranblue).linear(flowerHblue, flowerHtranblue);
    Path flowerHtranshootLblue = Paths.line(flowerHtranblue, shootLblue).linear(flowerHtranblue, shootLblue);


    Path shootLflowerHtranblue = Paths.line(shootLblue, flowerHtranblue).linear(shootLblue, flowerHtranblue);


    Path flowerHtranparkblue = Paths.line(flowerHtranblue, parkblue).constant(Math.toRadians(-90));


    private Path pathstartshootHblue() {
        return Paths.line(startPose, shootHblue).constant(Math.toRadians(90));
    }

    private Path pathshootHcornerblue() {
        return Paths.line(shootHblue, cornerblue).linear(shootHblue, cornerblue);
        // return line()
    }

    private Path pathcornershootLblue() {
        return Paths.path(corrnershootHblue, shootHshootLtransblue, shootLtransshootLblue);
    }

    private Path pathflowerLflowerHblue() {
        return Paths.path(flowerLflowerHtranblue, flowerHtranflowerHblue);
    }

    private Path pathflowerHshootLblue() {
        return Paths.path(flowerHflowerHtranblue, flowerHtranshootLblue);
    }

    private Path pathshootLparkblue() {
        return Paths.path(shootLflowerHtranblue, flowerHtranparkblue);
    }


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

       // currentH=Math.toDegrees(pose.heading());
        if(pose.y()>0) {
            if(colorRed) targetPose = rbga.targetPoseredh;
            else targetPose = rbga.targetPoseblueh;
        }
        else {
           // offset=180;
            if(colorRed) targetPose = rbga.targetPoseredl;
            else targetPose = rbga.targetPosebluel;
        }
//     //   if (dist == 0) dist = pose.distance(targetPose);
//        dist = pose.distance(targetPose)-5.4;

        targetflyvel=rbga.flySpeed(dist);
        targetH = Math.toDegrees(Math.atan2(pose.y()-targetPose.y() , pose.x()-targetPose.x() ));
        if(targetH<0) targetH=targetH+360;

        Hood.setPosition(rbga.flyHood(dist));
        flyWheel.setPower(rbga.flyPIDF(flyCurrentVel,targetflyvel) );

    }



    void Pathupdate1() {


        if(autotime.seconds()>89&&!complete) pathState=90;

        switch (pathState) {



            case 0:

                if(colorRed)
                    follower.follow(pathstartshootLred());
                else
                    follower.follow(pathstartshootHblue());

                pathState = 1;
                break;

            case 1:
                if(!follower.isBusy()) {
                    pathState = 2;
                    intakeBack.setPower(0.7);
                    intakeFront.setPower(0.9);
                }
                break;


            case 2:
                if(shoot()) {
                    rbga.timer(0);
                    pathState = 3;
                }
                break;


            case 3:
                if(rbga.timer(1500)) {
                    // flyWheel.setPower(0);
                    // intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    Blocker.setPosition(rbga.blockClose);

                    if(colorRed)
                        follower.follow(pathshootLcornerred());
                    else
                        follower.follow(pathshootHcornerblue());

                    pathState = 7;
                    rbga.timer(0);
                }
                break;


            case 7:
                if(!follower.isBusy()) {
                    follower.manual(0.3,0,0);
                    rbga.timer(0);
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    pathState = 9;
                }
                break;


//

            case 9:
                if(rbga.timer(1200)) {

                    if(colorRed)
                        follower.follow(pathcornershootHred());
                    else
                        follower.follow(pathcornershootLblue());

                    dist = 31.78;

                    intakeBack.setPower(0);
                    intakeFront.setPower(0);
                    pathState = 15;
                    Flipper.setPosition(rbga.flippermid);
                    // rbga.timer(0);
                }
                break;


            case 15:
                if(follower.distanceToEndpoint() < 80) {
                    Flipper.setPosition(rbga.flipperExtend);
                    pathState = 20;
                }
                break;


            case 20:
                if(!follower.isBusy()) {
                    pathState = 25;
                }
                break;


            case 25:
                if(shoot()) {
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    pathState = 30;
                    rbga.timer(0);
                }
                break;


            case 30:
                if(rbga.timer(1500)) {
                    Blocker.setPosition(rbga.blockClose);

                    follower.manual(0.3,0,0);
                    rbga.timer(0);
                    dist = 39;
                    pathState = 40;
                }
                break;


            case 40:
                if(rbga.timer(1000)) {
                    follower.manual(0,0,0);
                    pathState = 45;
                }
                break;


            case 45:
                if(shoot()) {
                    rbga.timer(0);
                    pathState = 50;
                }
                break;


            case 50:
                if(rbga.timer(2000)) {

                    Blocker.setPosition(rbga.blockClose);
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);

                    if(colorRed)
                        follower.follow(pathflowerHflowerLred());
                    else
                        follower.follow(pathflowerLflowerHblue());

                    Flipper.setPosition(rbga.flippermid);
                    dist = 31.78;
                    pathState = 51;
                }
                break;


            case 51:
                if(follower.distanceToEndpoint() < 40) {
                    Flipper.setPosition(rbga.flipperExtend);
                    pathState = 55;
                }
                break;


            case 55:
                if(!follower.isBusy()) {
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    follower.manual(0.3,0,0);
                    rbga.timer(0);
                    pathState = 60;
                }
                break;


            case 60:
                if(rbga.timer(1000)) {
                    follower.manual(0,0,0);
                    pathState = 65;
                }
                break;


            case 65:
                if(rbga.timer(2000)) {

                    if(colorRed)
                        follower.follow(pathflowerLshootHred());
                    else
                        follower.follow(pathflowerHshootLblue());

                    pathState = 67;
                }
                break;

            case 67:
                if(rbga.timer(2400)) {

                    Flipper.setPosition(rbga.flipperCollapse);
                    intakeBack.setPower(0);
                    intakeFront.setPower(0);
                    pathState = 70;
                }

            case 70:
                if(!follower.isBusy()) {
                    pathState = 75;
                }
                break;


            case 75:
                if(shoot()) {
                    intakeFront.setPower(0.9);
                    intakeBack.setPower(0.7);
                    rbga.timer(0);
                    pathState = 80;
                }
                break;


            case 80:
                if(rbga.timer(1500)) {

                    Blocker.setPosition(rbga.blockClose);
                    update = false;
                    flyWheel.setPower(0);
                    intakeBack.setPower(0);
                    intakeFront.setPower(0);

                    if(colorRed)
                        follower.follow(pathshootHparkred());
                    else
                        follower.follow(pathshootLparkblue());

                    pathState = 85;
                }
                break;


            case 85:
                if(!follower.isBusy()) {
                    rbga.timer(0);
                    follower.manual(0,0,0);
                    pathState = 100;
                    complete=true;
                }
                break;

            case 90:
                    rbga.timer(0);
                    follower.manual(0,0,0);
                    complete=true;
                    flyWheel.setPower(0);
                    intakeBack.setPower(0);
                    intakeFront.setPower(0);
                    pathState = 100;

                break;
            case 100:
                if(rbga.timer(700)) {
                    autosave();

                    pathState = 200;
                }
                break;


            case 200:
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