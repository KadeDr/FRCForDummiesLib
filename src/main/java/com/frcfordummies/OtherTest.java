package com.frcfordummies;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OtherTest {
    public SparkMax spark = new SparkMax(1, MotorType.kBrushless);
    public SparkMaxConfig config = new SparkMaxConfig();

    public void test() {
        config
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .p(1);
    }
}
