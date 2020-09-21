package org.firstinspires.ftc.teamcode.ACompetitionSkyStone.Controls.WoodBotControls;

import org.firstinspires.ftc.teamcode.ACompetitionSkyStone.robots.WoodBot;

public abstract class AutoBuildingWood extends AutoMainWood {



    public void alignBuildPlate (WoodBot Bot, String Alliance) {


        Bot.strafeLeft(highSpeed, 3);

        if (Alliance == "Red") {
            Bot.driveForward(midSpeed, 1);
        }
        else if (Alliance == "Blue") {
            Bot.driveBackward(midSpeed, 1);
            Bot.strafeLeft(lowSpeed, .4);
        }

        Bot.strafeLeft(lowSpeed, .5);

        Bot.HookGrab();
        sleep(1000);

        Bot.strafeRight(midSpeed, 4.8);

        Bot.HookRelease();

        Bot.gyroCorrection(gyroSPD, 0);

    }


    public void goToSkystones(WoodBot Bot, String Alliance) {
        if (Alliance == "Red") {
            Bot.gyroCorrection(gyroSPD, -1);

            Bot.driveBackward(highSpeed, 5.5);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.strafeLeft(highSpeed, 2.5);
            sleep(sleepTime);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.driveBackward(highSpeed, .8);
            sleep(sleepTime);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.strafeLeft(lowSpeed, .15);


        }

        else if (Alliance == "Blue") {

            Bot.gyroCorrection(lowSpeed, -1);

            Bot.driveForward(highSpeed, 5.5);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.strafeLeft(highSpeed, 2.5);
            sleep(sleepTime);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.driveForward(highSpeed, .8);
            sleep(sleepTime);

            Bot.gyroCorrection(gyroSPD, 0);

            Bot.strafeRight(highSpeed, .15);
            sleep(sleepTime);
        }

            

    }

    public void goToSkystonesOuter(WoodBot Bot, String Alliance) {
        if (Alliance == "Red") {
            Bot.gyroCorrection(gyroSPD, -1);

            Bot.driveBackward(highSpeed, 5.5);

            Bot.gyroCorrection(gyroSPD, 0);


        }

        else if (Alliance == "Blue") {

            Bot.gyroCorrection(lowSpeed, -1);

            Bot.driveForward(highSpeed, 5.5);

            Bot.gyroCorrection(gyroSPD, 0);

        }

//

    }

    public void dropStone (WoodBot Bot) {

            Bot.strafeLeft(highSpeed, 6);
            Bot.dropStone();


    }


    public void park (WoodBot Bot, String Alliance) {
            if (Alliance == "Red") {
                Bot.strafeRight(midSpeed, 2.5);

        }
            else if (Alliance == "Blue") {
                Bot.driveBackward(midSpeed, 2.5);
            }

    }



}
