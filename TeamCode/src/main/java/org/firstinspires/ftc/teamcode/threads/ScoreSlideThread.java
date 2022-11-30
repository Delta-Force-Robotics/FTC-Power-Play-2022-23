package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;

import java.util.concurrent.TimeUnit;

public class ScoreSlideThread extends  Thread{
    SlideInterface slideInterface;
    int slideLevelForScore;
    Timing.Timer timer;

    public ScoreSlideThread(SlideInterface slideInterface, int slideLevelForScore){
        this.slideInterface=slideInterface;
        this.slideLevelForScore=slideLevelForScore;
    }
@Override
    public void run(){
        slideInterface.setLevel(slideLevelForScore);
}
}
