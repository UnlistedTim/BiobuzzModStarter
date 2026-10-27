
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class base  {// to save common data and function.

    public final double ticksPerDegree =113.7777; // 114.02
    public double timerecord;


    private PIDController drivePID = new PIDController(0.015,0,0.0008);
    //   private PIDController drivePID = new PIDController(0.01,0,0.0008);

    public final double driveKs = 0.1;
   public ElapsedTime basetime = new ElapsedTime();

    boolean rotating = false;
    InterpLUT Flylut = new InterpLUT();
    InterpLUT Hoodlut = new InterpLUT();
    double dist = 0;
    public final double flyp = 0.012, flyi=0, flyd = 0, flyv = 0.00036, flys = 0.04;
    public final double hoodCollapse = 0.38, hoodExtend = 0.28, hoodShoot = 0.35;
    public final double flipperExtend = 0.38, flipperCollapse = 0.75;
    public final double blockClose = 0.63, blockOpen = 0.27;
    public boolean shooting = false, canShoot = false;
    public static final double  startX1 = -16.01 , startY1 = -55.49;
    public final double  startXred1=startX1, startYred1=startY1,startHeadingred1=-90;
    public final double startXblue1=-startX1,startYbue1=-startY1 ,startHeadingblue1=90;

     public final PoseFactory p = PoseFactory.degrees();
    public double targetX = -23+11, targetY = -70.6+55;
    public double targetXredl = targetX, targetYredl = targetY;
    public double targetXredh = targetX, targetYredh = -targetY;
    public double targetXbluel = -targetX, targetYbluel = targetY;
    public double targetXblueh = -targetX, targetYblueh = -targetY;
    public double targetHeading = 0,currentHeading=0;
    public Pose startPosered1 = p.of(startX1,startY1,-90);
    public Pose startPoseblue1 = p.of(-startX1,-startY1,90);


    public  Pose targetPose = p.of(targetX,targetY,-90);
    public  Pose targetPoseredl = p.of(targetXredl,targetYredl,180);
    public  Pose targetPoseredh = p.of(targetXredh,targetYredh,0);
    public  Pose targetPosebluel = p.of(targetXbluel,targetYbluel,180);
    public  Pose targetPoseblueh = p.of(targetXblueh,targetYblueh,0);

    public boolean colorRed=true;



    public double Xred = 58, Xb = Xred + 24;

    public double Yback = 54, Yfront = Yback + 2*18;
    public double posex,posey;



    public static double blockerPos = 0.27; //0.27 up 0.63 collapse

    public final int shootVel = 1180;
    public Follower follower;

    public double flyCurrentVel = 0, targetFlyVel = 0;
    public boolean outtakePhase=false,timer=false,aligNing=false,driVeable=true, intakeTimer = false, outtakeTimer = false;
    private boolean flipperFlag=false,recevieinfo=false;

    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};






   public Pose startpos(boolean red){

    Pose st;
    if(red) st=startPosered1;
    else  st=startPoseblue1;
   return st;

   }


    public double flooralign(double cheading ,double theading){


        double shootHeading =cheading%360;
        double turnPower=0;
        double error=theading-shootHeading ;

        if (Math.abs(error)< 2)return turnPower;
        if (Math.abs(error) > 180) error =-Math.signum(error) *Math.abs(360-Math.abs(error));// change rotation direction

        turnPower= drivePID.calculate(error);

        turnPower+= Math.signum(turnPower) * driveKs;

        return turnPower;

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



    public boolean timer(double period ) {

        if (period == 0) {
             timerecord= basetime.milliseconds();
            return false;
        }
        return basetime.milliseconds() - timerecord > period;

    }

    public double flySpeed(double dist){
        if (dist < 56.6 && dist > 20.7) {

            return Flylut.get(dist);// Tx offset

//            Tx_offset=0;

        }
        else return 1130;


    }
    public double flyHood(double dist){
        if (dist < 56.6 && dist > 20.7) {

            return Hoodlut.get(dist);

//            Tx_offset=0;

        }
        else return 0.34;//1500
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


}


