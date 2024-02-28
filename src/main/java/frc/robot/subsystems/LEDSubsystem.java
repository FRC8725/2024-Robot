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
    public static final int LED_BUFFER_LENGTH = 59;

  	public final AddressableLED led;
  	public final AddressableLEDBuffer ledBuffer;

    private boolean isIdle = true;
    
  	/** Creates a new LEDsubsystem. */
  	public LEDSubsystem() {
        this.led = new AddressableLED(RobotPorts.PWM.LED_PORT.get());
        this.ledBuffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

        this.led.setLength(LED_BUFFER_LENGTH);
        this.led.setData(this.ledBuffer);
        this.led.start();
  	}


	private int rainbowFirstPixelHue = 0;

    public void setIdleMode(boolean idleMode) {
        this.isIdle = idleMode;
    }

	public void rainbow() {
		for (var i = 0; i < LED_BUFFER_LENGTH; i++) {
			final var hue = (rainbowFirstPixelHue + (i * 180 / this.ledBuffer.getLength())) % 180;
			this.ledBuffer.setHSV(i, hue, 255, 128);
		}

		rainbowFirstPixelHue += 3;
		rainbowFirstPixelHue %= 180;
	}

    public void setColor(Color color) {
        for (var i = 0; i < LED_BUFFER_LENGTH; i++) {
            this.ledBuffer.setLED(i, color);
        }
    }

    public void setColorWithPercentage(Color color, Color backgroundColor, double percentage) {
        if (percentage > 1) percentage = 1;
        else if (percentage < 0) percentage = 0;
        
        int endLED = (int)(LED_BUFFER_LENGTH * percentage);

        for (var i = 0; i < endLED; i++) {
            this.ledBuffer.setLED(i, color);
        }

        for (var i = endLED; i < LED_BUFFER_LENGTH; i++) {
            this.ledBuffer.setLED(i, backgroundColor);
        }
    }

    public void setColorWithPercentageInRange(Color color, Color backgroundColor, double percentage, int start, int end) {
        if (percentage > 1) percentage = 1;
        else if (percentage < 0) percentage = 0;

        if (start < 0) start = 0;
        if (end > LED_BUFFER_LENGTH) end = LED_BUFFER_LENGTH;
        
        int endLED = (int)((end - start) * percentage);

        for (var i = start; i < endLED; i++) {
            this.ledBuffer.setLED(i, color);
        }

        for (var i = endLED; i < end; i++) {
            this.ledBuffer.setLED(i, backgroundColor);
        }
    }

    @Override
    public void periodic() {
        if (isIdle) rainbow();
        this.led.setData(this.ledBuffer);
    }
}
