package com.frcfordummies.rev.spark;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public abstract class MAXBase {
    public SparkMax spark;
    public AbsoluteEncoder absoluteEncoder;
    public RelativeEncoder relativeEncoder;
    public SparkClosedLoopController CLC;
    public SparkMaxConfig config;
    public MotorType motorType;
    public int id;

    
}
