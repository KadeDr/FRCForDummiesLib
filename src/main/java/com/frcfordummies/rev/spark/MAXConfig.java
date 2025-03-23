package com.frcfordummies.rev.spark;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MAXConfig extends MAXBase{
    public MAXConfig() {}

    public final ClosedLoopConfig pid = this.config.closedLoop;

    public MAXConfig idleMode(IdleMode idleMode) {
        this.config.idleMode(idleMode);
        return this;
    }

    public ClosedLoopConfig p(double value) {
        pid.p(value);
        return pid;
    }

    public ClosedLoopConfig i(double value) {
        pid.i(value);
        return pid;
    }

    public ClosedLoopConfig d(double value) {
        pid.d(value);
        return pid;
    }

    public ClosedLoopConfig encoderType(FeedbackSensor encoder) {
        pid.feedbackSensor(encoder);
        return pid;
    }
}
