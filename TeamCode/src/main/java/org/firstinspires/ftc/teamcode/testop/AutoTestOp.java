package org.firstinspires.ftc.teamcode.testop;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Autonomous(name="Test Op")
public class AutoTestOp extends LinearOpMode {

    @Override
    public void runOpMode()throws InterruptedException{
        Mecanum drive = new MecanumRevOptimized(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TrajectoryBuilder builder = new TrajectoryBuilder(
                new Pose2d(-59, 59, -50),
                Mecanum.CONSTRAINTS
        );

        builder
                .splineTo(new Pose2d(-30, -32, -50), new ConstantInterpolator(-50))
                .splineTo(new Pose2d(30, -52, 0), new LinearInterpolator(-50, 0))
                .turnTo(90);

        Trajectory trajectory = builder.build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while(!isStopRequested() && drive.isFollowingTrajectory()){
            Pose2d currentPose = drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            dashboard.sendTelemetryPacket(packet);

            drive.update();
        }

    }
}
