package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;

public class ScoreSlideThread extends  Thread{
    SlideInterface slideSubsystem;
    int slideLevelForScore;
    Timing.Timer timer;

    public ScoreSlideThread(SlideInterface slideSubsystem, int slideLevelForScore){
        this.slideSubsystem = slideSubsystem;
        this.slideLevelForScore = slideLevelForScore;
    }

    @Override
    public void run(){
        slideSubsystem.setLevel(slideLevelForScore, false);
    }
}
