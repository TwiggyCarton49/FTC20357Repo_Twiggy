package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class autoTest2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(24, -61, Math.toRadians(90)))
                        .strafeTo(new Vector2d(26,-59))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-25,-15,Math.toRadians(0)),Math.toRadians(90))
                        .waitSeconds(6)
                        .lineToX(-26)
                        .setTangent(Math.toRadians(268))
                        .splineToConstantHeading(new Vector2d(56,-62),Math.toRadians(0))

                .build());
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(24,61,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(25,15,Math.toRadians(180)),Math.toRadians(180))
                .waitSeconds(6)
                .lineToX(26)
                .setTangent(Math.toRadians(3))
                .splineToConstantHeading(new Vector2d(-56,62),Math.toRadians(179))
                .build());

        RoadRunnerBotEntity bot3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot3.runAction(bot3.getDrive().actionBuilder(new Pose2d(-24, -61, Math.toRadians(90)))
                        .setTangent(Math.toRadians(85))
                        .splineToLinearHeading(new Pose2d(-25,15,Math.toRadians(0)),Math.toRadians(25))
                        .waitSeconds(6)
                        .lineToX(-35)
                        .setTangent(Math.toRadians(235))
                        .splineToConstantHeading(new Vector2d(56,-62),Math.toRadians(0))
                        .build());
        RoadRunnerBotEntity bot4 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot4.runAction(bot4.getDrive().actionBuilder(new Pose2d(-24, 61, Math.toRadians(270)))
                        .setTangent(Math.toRadians(350))
                        .splineToLinearHeading(new Pose2d(25,-15,Math.toRadians(180)),Math.toRadians(180))
                        .waitSeconds(6)
                        .lineToX(34)
                        .setTangent(45)
                        .splineToConstantHeading(new Vector2d(-56,62),Math.toRadians(180))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(bot2)
                .addEntity(bot3)
                .addEntity(bot4)
                .start();


    }
}

