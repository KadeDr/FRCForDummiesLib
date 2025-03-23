package com.frcfordummies;

import com.frcfordummies.rev.spark.MAX;
import com.frcfordummies.rev.spark.MAXConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Test {
    public MAX spark = new MAX(1);
    public MAXConfig config = new MAXConfig();

    public void test() {
        spark.BrushlessMotor();
        spark.setPosition(1);
        config
            .idleMode(IdleMode.kBrake);
        config.pid
            .p(0)
            .i(0)
            .d(0);
    }
}
