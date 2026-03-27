package simulator;

import fr.emse.fayol.maqit.simulator.components.ColorSituatedComponent;
import fr.emse.fayol.maqit.simulator.components.ComponentType;

public class ColorRechargeZone extends ColorSituatedComponent {

    public ColorRechargeZone(int[] pos, int[] rgb) {
        super(pos, rgb);
    }

    @Override
    public ComponentType getComponentType() {
        return ComponentType.unknown;
    }
}
