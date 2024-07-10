// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.RainbowAnimation;

public class Lightning extends SubsystemBase {
    private CANdle candle;
    private Animation animation = null;
    private int num_leds;
    
    /** Creates a new leds. */
    public Lightning(int id, int num_leds) {
        this.num_leds = num_leds;
        this.candle = new CANdle(id);

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.brightnessScalar = 0.5;
        cfg.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(cfg);
        candle.configLEDType(LEDStripType.GRB);
        setDisabledLightShow();
    }

    public void setTeleOpLightShow() {
        animation = new TwinkleAnimation(255, 0, 255, 0, 1, this.num_leds, TwinklePercent.Percent100);
    }

    public void setDisabledLightShow() {
        animation = new RainbowAnimation(0.1, 0.6, this.num_leds);
    }

    public void setVisionTrackingLightShow() {
        animation = new TwinkleAnimation(0, 255, 0, 0, 1, this.num_leds, TwinklePercent.Percent100);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (animation != null){
            candle.animate(animation);
        }
    }
}
