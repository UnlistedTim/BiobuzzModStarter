package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class Autopath extends LinearOpMode {
    private double startX = -16.01 , startY = -55.49;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    public  Pose startPose = poseFactory.of(startX,startY,-90);

    private final Pose start = poseFactory.of(60.899, 10.3485, 90);
    private final Pose path1 = poseFactory.of(14.3361, 104.0094, 270);
    private final Pose parkPosered1 = poseFactory.of(-47, 27, -90);
    private final Pose parkPosectlred1 = poseFactory.of(-47, -47, -90);
    private final Pose path1Control1 = poseFactory.of(18, 24, 0);
    private Path parkred1(){
        return Paths.curve(startPose, parkPosectlred1,parkPosered1).constant(Math.toRadians(-90));
        // return line()
    }
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, parkred1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(Math.toRadians(-90));
    }
}