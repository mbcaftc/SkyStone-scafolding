package org.firstinspires.ftc.teamcode.CompititionUltimateGoal.Controls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompititionUltimateGoal.Robots.LabBot;

@Autonomous(name = "AutoBotsRollOut")
public class AutoBotsStrafe extends LinearOpMode {

    public LabBot Bot = new LabBot();

    @Override
    public void runOpMode() throws InterruptedException {


        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        waitForStart();

        while (opModeIsActive()) {
            Bot.driveForward(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.strafeLeft(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.driveBackward(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.strafeRight(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            requestOpModeStop();

        }
        idle();
    }
}
