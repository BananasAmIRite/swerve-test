package frc.robot.lib.leds.addressable;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.leds.addressable.patterns.LEDPattern;

public class LED extends SubsystemBase {

    private AddressableLED led; 

    private LEDStrip ledStrip;

    private AddressableLEDBuffer buffer; 

    private LEDPattern pattern; 

    private Timer timer = new Timer();

    public LED(int port, int length, LEDPattern pattern) {
        this.led = new AddressableLED(port); 
        this.led.setLength(length);
        this.buffer = new AddressableLEDBuffer(length); 

        this.timer = new Timer();

        this.timer.start();
        this.led.start();

        setPattern(pattern);
    }

    public void setPattern(LEDPattern pattern) {
        if (this.pattern == pattern) return; 
        ledStrip.setPattern(transformPattern(pattern));
        this.pattern = pattern; 
        this.timer.restart();
    }

    // this is where you could use a SplitPattern to split the led and stuff
    protected LEDPattern transformPattern(LEDPattern pattern) {
        return pattern;
    }

    @Override
    public void periodic() {
        ledStrip.update(timer.get()); 
        led.setData(buffer);
    }

    public AddressableLED getLed() {
        return this.led; 
    }
}