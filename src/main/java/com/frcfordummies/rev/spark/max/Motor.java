package com.frcfordummies.rev.spark.max;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

public class Motor extends MAXBase {

    /**
     * 
     * @param id        The CAN ID on the Spark MAX. You can find this in the Rev
     *                  Hardware Client.. or through Trial and Error if you really
     *                  want.
     * @param motorType The type of motor (brushed, brushless). You can find this
     *                  information online.
     */
    public Motor(int id) {
        this.id = id;
        this.spark = new SparkMax(this.id, null);
        this.absoluteEncoder = spark.getAbsoluteEncoder();
        this.relativeEncoder = spark.getEncoder();
        this.CLC = spark.getClosedLoopController();
    }

    public void setBrushed() {
        motorType = MotorType.kBrushed;
        spark = new SparkMax(this.id, this.motorType);
    }

    public void setBrushless() {
        motorType = MotorType.kBrushless;
        spark = new SparkMax(this.id, this.motorType);
    }

    /**
     * 
     * @param speed The speed you want your motor to move at. Range between -1 and
     *              1.
     */
    public void setVelocity(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        spark.set(speed);
    }

    /**
     * 
     * @param position The position you are trying to go to in meters
     * @param rpm The max RPM of the motor you are using
     * @param gearRatio The gear ratio on the motor
     * @param maxSpeed The max speed of the motor. Range between 0 and 1.
     */
    public REVLibError setPositionMeters(double position, double rpm, double gearRatio, double maxSpeed) {
        maxSpeed = MathUtil.clamp(maxSpeed, 0, 1);
        config
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(0.004064)
            .velocityConversionFactor((rpm / gearRatio) * 0.0127 * Math.PI / 60);
        config.closedLoop
            .pid(0.15, 0, 0)
            .outputRange(-maxSpeed, maxSpeed);

        return CLC.setReference(position, ControlType.kPosition);
    }

    /**
     * 
     * @param position The position you are trying to go to in inches
     * @param rpm The max RPM of the motor you are using
     * @param gearRatio The gear ratio on the motor
     * @param maxSpeed The max speed of the motor. Range between 0 and 1.
     */
    public REVLibError setPositionInches(double position, double rpm, double gearRatio, double maxSpeed) {
        maxSpeed = MathUtil.clamp(maxSpeed, 0, 1);
        config
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(0.16)
            .velocityConversionFactor((rpm / gearRatio) * 0.5 * Math.PI / 60);
        config.closedLoop
            .pid(0.15, 0, 0)
            .outputRange(-maxSpeed, maxSpeed);

        return CLC.setReference(position, ControlType.kPosition);
    }
}
