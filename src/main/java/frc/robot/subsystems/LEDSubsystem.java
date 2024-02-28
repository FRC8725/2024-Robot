// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotPorts;

public class LEDSubsystem extends SubsystemBase {
    public static final int TELESCOPE_LED_BUFFER_LENGTH = 32;
    public static final int TELESCOPE_LED_BUFFER_EACH_LENGTH = 16;

  	public final AddressableLED telescopeLED;
  	public final AddressableLEDBuffer telescopeLEDBuffer;

    private boolean isIdle = true;

  	public LEDSubsystem() {
        this.telescopeLED = new AddressableLED(RobotPorts.PWM.LEFT_TELESCOPE_LED_PORT.get());
        this.telescopeLEDBuffer = new AddressableLEDBuffer(TELESCOPE_LED_BUFFER_LENGTH);

        this.telescopeLED.setLength(TELESCOPE_LED_BUFFER_LENGTH);
        this.telescopeLED.setData(this.telescopeLEDBuffer);
        this.telescopeLED.start();
  	}


	private int rainbowFirstPixelHue = 0;

    public void setIdleMode(boolean idleMode) {
        this.isIdle = idleMode;
    }

	public void rainbow() {
		for (var i = 0; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
			final var hue = (rainbowFirstPixelHue + (i * 180 / this.telescopeLEDBuffer.getLength())) % 180;
			this.telescopeLEDBuffer.setHSV(i, hue, 255, 128);
            this.telescopeLEDBuffer.setHSV(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), hue, 255, 128);
        }
		rainbowFirstPixelHue += 1;
		rainbowFirstPixelHue %= 180;
	}

    public void setAllColor(Color color) {
        for (var i = 0; i < TELESCOPE_LED_BUFFER_LENGTH; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
        }
    }

    public void setColorInRange(Color backgroundColor, Color color, int start, int end) {
        if (start < 0) start = 0;
        if (end > TELESCOPE_LED_BUFFER_EACH_LENGTH) end = TELESCOPE_LED_BUFFER_EACH_LENGTH;

        for (var i = 0; i < start; i++) {
            this.telescopeLEDBuffer.setLED(i, backgroundColor);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
        }

        for (var i = start; i < end; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), color);
        }

        for (var i = end; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
            this.telescopeLEDBuffer.setLED(i, backgroundColor);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
        }
    }

    public void setColorWithPercentage(Color backgroundColor, Color color, double percentage) {
        if (percentage > 1) percentage = 1;
        else if (percentage < 0) percentage = 0;
        
        int endLED = (int)(TELESCOPE_LED_BUFFER_EACH_LENGTH * percentage);

        for (var i = 0; i < endLED; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), color);
        }

        for (var i = endLED; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
            this.telescopeLEDBuffer.setLED(i, backgroundColor);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
        }
    }

    public void setColorWithPercentageInRange(Color backgroundColor, Color color, double percentage, int start, int end) {
        if (percentage > 1) percentage = 1;
        else if (percentage < 0) percentage = 0;

        if (start < 0) start = 0;
        if (end > TELESCOPE_LED_BUFFER_EACH_LENGTH) end = TELESCOPE_LED_BUFFER_EACH_LENGTH;
        
        int endLED = (int)((end - start) * percentage);

        for (var i = start; i < endLED; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), color);
        }

        for (var i = endLED; i < end; i++) {
            this.telescopeLEDBuffer.setLED(i, backgroundColor);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
        }
    }

    @Override
    public void periodic() {
        if (isIdle) rainbow();
        this.telescopeLED.setData(this.telescopeLEDBuffer);
    }
}
