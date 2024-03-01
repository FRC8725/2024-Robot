package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotPorts;

public class LEDSubsystem extends SubsystemBase {
    public static final int TELESCOPE_LED_BUFFER_EACH_LENGTH = 16;
    private static final int TELESCOPE_LED_BUFFER_LENGTH = 32;

  	private final AddressableLED telescopeLED;
  	private final AddressableLEDBuffer telescopeLEDBuffer;
    private boolean isIdle = true;
    private int rainbowFirstPixelHue = 0;

  	public LEDSubsystem() {
        this.telescopeLED = new AddressableLED(RobotPorts.PWM.LEFT_TELESCOPE_LED_PORT.get());
        this.telescopeLEDBuffer = new AddressableLEDBuffer(TELESCOPE_LED_BUFFER_LENGTH);

        this.telescopeLED.setLength(TELESCOPE_LED_BUFFER_LENGTH);
        this.telescopeLED.setData(this.telescopeLEDBuffer);
        this.telescopeLED.start();
  	}

    public void setIdleMode(boolean idleMode) {
        this.isIdle = idleMode;
    }

	private void rainbow() {
		for (int i = 0; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
			final int hue = (this.rainbowFirstPixelHue + (i * 180 / this.telescopeLEDBuffer.getLength())) % 180;
			this.telescopeLEDBuffer.setHSV(i, hue, 255, 128);
            this.telescopeLEDBuffer.setHSV(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), hue, 255, 128);
        }

		this.rainbowFirstPixelHue += 1;
		this.rainbowFirstPixelHue %= 180;
	}

    public void setAllColor(Color color) {
        for (int i = 0; i < TELESCOPE_LED_BUFFER_LENGTH; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
        }
    }

    public void setColorInRange(Color backgroundColor, Color color, int start, int end) {
        if (start < 0) start = 0;
        if (end > TELESCOPE_LED_BUFFER_EACH_LENGTH) end = TELESCOPE_LED_BUFFER_EACH_LENGTH;
        if (start > end) return;

        for (int i = 0; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
            if (i >= start && i < end) {
                this.telescopeLEDBuffer.setLED(i, color);
                this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), color);
            } else {
                this.telescopeLEDBuffer.setLED(i, backgroundColor);
                this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
            }
        }
    }

    public void setColorWithPercentage(Color backgroundColor, Color color, double percentage) {
        if (percentage > 1) percentage = 1;
        else if (percentage < 0) percentage = 0;
        
        int endLED = (int) (TELESCOPE_LED_BUFFER_EACH_LENGTH * percentage);

        for (int i = 0; i < endLED; i++) {
            this.telescopeLEDBuffer.setLED(i, color);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), color);
        }

        for (int i = endLED; i < TELESCOPE_LED_BUFFER_EACH_LENGTH; i++) {
            this.telescopeLEDBuffer.setLED(i, backgroundColor);
            this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (i + 1), backgroundColor);
        }
    }

    private void setColorBothSide(Color color, int index) {
          this.telescopeLEDBuffer.setLED(index, color);
          this.telescopeLEDBuffer.setLED(TELESCOPE_LED_BUFFER_LENGTH - (index + 1), color);
    }

    @Override
    public void periodic() {
        if (isIdle) rainbow();
        this.telescopeLED.setData(this.telescopeLEDBuffer);
    }
}
