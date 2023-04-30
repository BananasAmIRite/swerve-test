package frc.robot.lib.util.tune;

public abstract class PIDTuner extends Tuner {
    private double p;
    private double i;
    private double d;

    private double setpoint;

    public PIDTuner(String name) {
        this(name, 0, 0, 0);
    }

    public PIDTuner(String name, double initialP, double initialI, double initialD) {
        super(name);
        this.p = initialP;
        this.i = initialI;
        this.d = initialD;
        addTunableDouble("P", (p) -> this.p = p);
        addTunableDouble("I", (i) -> this.i = i);
        addTunableDouble("D", (d) -> this.d = d);
        addDisplayValue("measurement", this::getMeasurement);
        addTunableDouble("setpoint", (setpoint) -> this.setpoint = setpoint);
    }

    public abstract void supplyPIDs(double p, double i, double d, double setpoint);

    public abstract double getMeasurement();

    @Override
    public void update() {
        super.update();
        supplyPIDs(p, i, d, setpoint);
    }
}
