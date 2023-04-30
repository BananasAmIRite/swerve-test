package frc.robot.lib.util.tune;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.*;

public abstract class Tuner {
    // TODO: these bindings kinda suck so maybe fix them
    // TODO: test
    private final Map<String, DisplayValue<?>> displayValues = new HashMap<>();
    private final Map<String, TunableValue<?>> tuneValues = new HashMap<>();
    private final String name;

    public Tuner(String name) {
        this.name = name;
    }

    public <T> void addDisplayValue(String name, Supplier<T> supp) {
        this.displayValues.put(name, new DisplayValue<>(getDisplayName(name), supp));
    }

    public void addTunableDouble(String name, Consumer<Double> onTune) {
        this.tuneValues.put(name, new DoubleTunableValue(getDisplayName(name), onTune));
    }
    public void addTunableDouble(String name, Consumer<Double> onTune, Double initialValue) {
        this.tuneValues.put(name, new DoubleTunableValue(getDisplayName(name), initialValue, onTune));
    }

    public void addTunableString(String name, Consumer<String> onTune) {
        this.tuneValues.put(name, new StringTunableValue(getDisplayName(name), onTune));
    }
    public void addTunableString(String name, Consumer<String> onTune, String initialValue) {
        this.tuneValues.put(name, new StringTunableValue(getDisplayName(name), initialValue, onTune));
    }

    public void addTunableBoolean(String name, Consumer<Boolean> onTune) {
        this.tuneValues.put(name, new BooleanTunableValue(getDisplayName(name), onTune));
    }
    public void addTunableBoolean(String name, Consumer<Boolean> onTune, Boolean initialValue) {
        this.tuneValues.put(name, new BooleanTunableValue(getDisplayName(name), initialValue, onTune));
    }

    public void update() {
        this.displayValues.forEach((a, b) -> b.update());
        this.tuneValues.forEach((a, b) -> b.update());
    }

    private String getDisplayName(String value) {
        return this.name + " - " + value;
    }


    private static class DisplayValue<T> {
        private final Supplier<T> supplier;
        private final String name;

        public DisplayValue(String name, Supplier<T> supplier) {
            this.supplier = supplier;
            this.name = name;
            init();
        }

        private void init() {
            update();
        }

        public T get() {
            return supplier.get();
        }

        public void update() {
            SmartDashboard.putString(name, get().toString());
        }
    }

    private static class TunableValue<T> {
        private final T initialValue;
        private final Consumer<T> onTune;
        private final String name;

        private T currentValue;

        private final BiFunction<String, T, T> getValue;
        private final BiConsumer<String, T> setValue;
        private final BiFunction<T, T, Boolean> equalValue;

        public TunableValue(String name, Consumer<T> onTune, BiFunction<String, T, T> getValue, BiConsumer<String, T> setValue, BiFunction<T, T, Boolean> equalValue) {
            this(name, getValue.apply(name, null), onTune, getValue, setValue, equalValue);
        }

        public TunableValue(String name, T initialValue, Consumer<T> onTune, BiFunction<String, T, T> getValue, BiConsumer<String, T> setValue, BiFunction<T, T, Boolean> equalValue) {
            this.initialValue = initialValue;
            this.onTune = onTune;
            this.getValue = getValue;
            this.setValue = setValue;
            this.equalValue = equalValue;
            this.name = name;
            init();
        }

        private void init() {
            setValue.accept(name, this.initialValue);;
        }

        public void update() {
            T newVal = getValue.apply(name, this.initialValue);

            if (!equalValue.apply(newVal, currentValue)) {
                this.currentValue = newVal;
                onTune.accept(newVal);
            }
        }
    }

    private static class StringTunableValue extends TunableValue<String> {
        public StringTunableValue(String name, Consumer<String> onTune) {
            super(name, onTune,
                    SmartDashboard::getString, SmartDashboard::putString, String::equals);
        }
        public StringTunableValue(String name, String initialValue, Consumer<String> onTune) {
            super(name, initialValue, onTune,
                    SmartDashboard::getString, SmartDashboard::putString, String::equals);
        }
    }

    private static class DoubleTunableValue extends TunableValue<Double> {
        public DoubleTunableValue(String name, Consumer<Double> onTune) {
            super(name, onTune,
                    SmartDashboard::getNumber, SmartDashboard::putNumber, Double::equals);
        }
        public DoubleTunableValue(String name, Double initialValue, Consumer<Double> onTune) {
            super(name, initialValue, onTune,
                    SmartDashboard::getNumber, SmartDashboard::putNumber, Double::equals);
        }
    }

    private static class BooleanTunableValue extends TunableValue<Boolean> {
        public BooleanTunableValue(String name, Consumer<Boolean> onTune) {
            super(name, onTune,
                    SmartDashboard::getBoolean, SmartDashboard::putBoolean, Boolean::equals);
        }
        public BooleanTunableValue(String name, Boolean initialValue, Consumer<Boolean> onTune) {
            super(name, initialValue, onTune,
                    SmartDashboard::getBoolean, SmartDashboard::putBoolean, Boolean::equals);
        }
    }
}
