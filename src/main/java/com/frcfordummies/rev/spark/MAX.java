package com.frcfordummies.rev.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class MAX extends MAXBase{

    /**
     * 
     * @param id        The CAN ID on the Spark MAX. You can find this in the Rev
     *                  Hardware Client.. or through Trial and Error if you really
     *                  want.
     * @param motorType The type of motor (brushed, brushless). You can find this
     *                  information online.
     */
    public MAX(int id) {
        this.id = id;
        this.spark = new SparkMax(this.id, null);
        this.absoluteEncoder = spark.getAbsoluteEncoder();
        this.relativeEncoder = spark.getEncoder();
        this.CLC = spark.getClosedLoopController();
    }

    public void BrushedMotor() {
        motorType = MotorType.kBrushed;
        spark = new SparkMax(this.id, this.motorType);
    }

    public void BrushlessMotor() {
        motorType = MotorType.kBrushless;
        spark = new SparkMax(this.id, this.motorType);
    }

    /**
     * 
     * @param value The position you want the motor to move to.
     * @return
     */
    public REVLibError setPosition(double value) {
        return CLC.setReference(value, ControlType.kPosition);
    }

    /**
     * 
     * @param value The position you want the motor to move to.
     * @param slot  Specify the slot if you have different slots at different speeds
     *              or measurements. Look at SparkConfig for more information.
     * @return
     */
    public REVLibError setPosition(double value, ClosedLoopSlot slot) {
        return CLC.setReference(value, ControlType.kPosition, slot);
    }

    /**
     * 
     * @param value The velocity you want the motor to spin at.
     * @return
     */
    public REVLibError setVelocity(double value) {
        return CLC.setReference(value, ControlType.kVelocity);
    }

    /**
     * 
     * @param value The velocity you want the motor to spin at.
     * @param slot  Specify the slot if you have different slots at different speeds
     *              or measurements. Look at SparkConfig for more information.
     * @return
     */
    public REVLibError setVelocity(double value, ClosedLoopSlot slot) {
        return CLC.setReference(value, ControlType.kVelocity, slot);
    }
}
