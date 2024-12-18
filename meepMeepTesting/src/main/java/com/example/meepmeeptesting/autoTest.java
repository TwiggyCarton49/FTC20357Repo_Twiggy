package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class autoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity RightBot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();


        RoadRunnerBotEntity RightBot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        RoadRunnerBotEntity  RightBot3= new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();


        RoadRunnerBotEntity  LeftBot1= new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        RoadRunnerBotEntity  LeftBot2= new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        RightBot1.runAction(RightBot1.getDrive().actionBuilder(new Pose2d(9, -61, Math.toRadians(90)))

                .strafeTo(new Vector2d(8, -36))
                //Hang Spec 1
                .strafeTo(new Vector2d(8,-40))
                .strafeTo(new Vector2d(31, -38))
                .strafeTo(new Vector2d(40, -12))
                .strafeTo(new Vector2d(46, -52))//push spec 2
                .strafeTo(new Vector2d(48, -15))
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(55, -52))//push spec 3
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(62, -15))
                .strafeTo(new Vector2d(62, -55))//push spec 4
                //pick up spec 2
                .strafeTo(new Vector2d(5, -35))
                //Hang Spec 2
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 3
                .strafeTo(new Vector2d(3, -35))
                //Hang spec 3
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 4
                .strafeTo(new Vector2d(0, -35))
                //Hang spec 4
                //END 4 sample Auto!!!!!!!

//--------------------------------------------------------------------------------------------------
                //Optional 5 Spec Auto
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 5
                .strafeTo(new Vector2d(-2, -35))
                //Hang spec 5
//--------------------------------------------------------------------------------------------------

                .strafeTo(new Vector2d(39, -58))//park

                .build());


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        RightBot2.runAction(RightBot2.getDrive().actionBuilder(new Pose2d(9, -61, Math.toRadians(90)))
                .strafeTo(new Vector2d(8, -36))
                //Hang Spec 1
                .strafeTo(new Vector2d(8,-40))
                .strafeTo(new Vector2d(30, -38))
                //.strafeTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(41,-12), Math.toRadians(0))
                .strafeTo(new Vector2d(46, -55))//push spec 3
                //pick up spec 2
                .strafeTo(new Vector2d(0, -35))
                .strafeTo(new Vector2d(20,-38))
                //Hang spec 2
                //.strafeTo(new Vector2d(55, -15))
                .splineToConstantHeading(new Vector2d(55,-15), Math.toRadians(0))

                .strafeTo(new Vector2d(55, -52))//push spec 4
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(62, -15))
                .strafeTo(new Vector2d(62, -55))//push spec 5
                //pick up spec 3
                .strafeTo(new Vector2d(5, -35))
                //Hang Spec 3
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 4
                .strafeTo(new Vector2d(3, -35))
                //Hang spec 4
                //END 4 sample Auto!!!!!!!

//--------------------------------------------------------------------------------------------------
                //Optional 5 Spec Auto
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 5
                .strafeTo(new Vector2d(-2, -35))
                //Hang spec 5
//--------------------------------------------------------------------------------------------------

                .strafeTo(new Vector2d(39, -58))//park

                .build());
//---------------------------------------------------------------------------------------------------------------------------------------------------------------//

        RightBot3.runAction(RightBot3.getDrive().actionBuilder(new Pose2d(9, -61, Math.toRadians(90)))//newest iteration of spec auto
                .strafeTo(new Vector2d(8, -36))
                //Hang Spec 1
                //.strafeTo(new Vector2d(8,-40))
                .strafeTo(new Vector2d(30, -38))
                //.strafeTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(41,-12), Math.toRadians(0))
                .strafeTo(new Vector2d(46, -55))//push spec 3
                //pick up spec 2
                .strafeTo(new Vector2d(0, -35))
                //Hang spec 2
                .strafeTo(new Vector2d(20,-38))
                .splineToConstantHeading(new Vector2d(55,-14), Math.toRadians(270))
                .strafeTo(new Vector2d(55, -55))//push spec 4
                //pick up spec 3
                .strafeTo(new Vector2d(5, -35))
                //Hang spec 3
                .strafeTo(new Vector2d(20,-38))
                .splineToConstantHeading(new Vector2d(62,-14), Math.toRadians(270))
                .strafeTo(new Vector2d(62, -55))//push spec 5
                //Pick up spec 4
                .strafeTo(new Vector2d(0, -35))
                //Hang spec 4

                //END 4 sample Auto!!!!!!!

//--------------------------------------------------------------------------------------------------
                //Optional 5 Spec Auto
                //.strafeTo(new Vector2d(40, -50))
                //.strafeTo(new Vector2d(40, -55))
                .splineToConstantHeading(new Vector2d(40,-60), Math.toRadians(270))
                //pick up spec 5
                .strafeTo(new Vector2d(-2, -35))
                //Hang spec 5

//--------------------------------------------------------------------------------------------------

                //.strafeTo(new Vector2d(39, -58))//park

                .build());

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


        LeftBot1.runAction(LeftBot1.getDrive().actionBuilder(new Pose2d(-12, -63, Math.toRadians(0)))//Use if team has 5 spec auto

                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(30,-60), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(30,-60), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-48,-40), Math.toRadians(90))
                //Pick up sample 1
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place sample 1
                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
                //Pick up sample 2
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place sample 2
                .strafeToLinearHeading(new Vector2d(-54,-25), Math.toRadians(180))
                //Pick up Sample 3
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place Sample 3
                .strafeToLinearHeading(new Vector2d(-23, -12), Math.toRadians(0))


                .build());

        LeftBot2.runAction(LeftBot2.getDrive().actionBuilder(new Pose2d(-12, -63, Math.toRadians(0)))//Use if team only has mediocre/bad spec auto (4 spec or less)


                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(30,-60), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-48,-40), Math.toRadians(90))
                //Pick up sample 1
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place sample 1
                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
                //Pick up sample 2
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place sample 2
                .strafeToLinearHeading(new Vector2d(-54,-25), Math.toRadians(180))
                //Pick up Sample 3
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                //Place Sample 3
                .strafeToLinearHeading(new Vector2d(-23, -12), Math.toRadians(0))


                .build());


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities

                .addEntity(RightBot1)
                //.addEntity(RightBot2)
                .addEntity(RightBot3)
                //.addEntity(LeftBot1)
                //.addEntity(LeftBot2)


                .start();


    }
}

