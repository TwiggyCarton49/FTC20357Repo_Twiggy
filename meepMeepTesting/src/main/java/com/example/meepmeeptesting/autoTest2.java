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
        MeepMeep meepMeep = new MeepMeep(600);
//------------------------------------------------------------------------------------------------------------------------------------
// **********BOTTOM LEFT************
        RoadRunnerBotEntity bot3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        bot3.runAction(bot3.getDrive().actionBuilder(new Pose2d(-9, -63, Math.toRadians(90)))
//                .strafeTo(new Vector2d(0, -52))
//                //April Tag detect
//                .waitSeconds(2)
                .strafeTo(new Vector2d(-9,-35))
                //Place Specimen
                .waitSeconds(2)
                .strafeTo(new Vector2d(-48,-40))
                //Pick up sample 1
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place sample 1
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
                //Pick up sample 2
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place sample 2
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-54,-25), Math.toRadians(180))
                //Pick up Sample 3
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place Sample 3
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-23, -12), Math.toRadians(0))
//                         .strafeTo(new Vector2d(-10,-35))
//                         .waitSeconds(3)
//                         .lineToY(-40)
//                         .splineToConstantHeading(new Vector2d(-48,-40),Math.toRadians(90))
//                        .waitSeconds(2)
//                        .splineToLinearHeading(new Pose2d(-55,-55,Math.toRadians(225)),Math.toRadians(180))
//                        .waitSeconds(2)
//                        .splineToLinearHeading(new Pose2d(-58,-40,Math.toRadians(90)),Math.toRadians(90))
//                        .waitSeconds(2)
//                        .splineToLinearHeading(new Pose2d(-55,-55,Math.toRadians(225)),Math.toRadians(180))

                        .build());
//------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(bot3)
                .start();


    }
}

