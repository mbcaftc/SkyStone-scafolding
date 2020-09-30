package org.firstinspires.ftc.teamcode.CompititionUltimateGoal.Controls.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompititionUltimateGoal.Robots.LabBot;

@Autonomous(name = "Bot:LabBot Auto:AutoBotRoll")
//@Disabled
public class AutoBotRoll extends LinearOpMode {

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
            Bot.rotateLeft(0.5);
            sleep(450);
            Bot.stopMotors();
            Bot.driveForward(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.rotateLeft(0.5);
            sleep(450);
            Bot.stopMotors();
            Bot.driveForward(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.rotateLeft(0.5);
            sleep(450);
            Bot.stopMotors();
            Bot.driveForward(0.5);
            sleep(1000);
            Bot.stopMotors();
            sleep(1000);
            Bot.rotateLeft(0.5);
            sleep(450);
            Bot.stopMotors();



            requestOpModeStop();
        }
        idle();


    }
}
