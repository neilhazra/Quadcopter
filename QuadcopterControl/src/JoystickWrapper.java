import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.Controller.Type;
import net.java.games.input.ControllerEnvironment;


public class JoystickWrapper {
    private static final String controllerName = "Logitech Dual Action";
    private Controller control = null;


    public JoystickWrapper() {
        control = getLogitech();
    }

    private static Controller getLogitech() {
        Controller[] controllers = ControllerEnvironment
                .getDefaultEnvironment().getControllers();
        for (Controller c : controllers) {
            if (c.getType() == Type.STICK && c.getName().equals(controllerName)) return c;
        }
        System.out.println("Joystick Wrapper: Plug in Logitech.");
        System.exit(0);
        return null;
    }

    public double getX() {
        net.java.games.input.Component x = control.getComponent(Identifier.Button._0);
        return x.getPollData();
    }

    public double getY() {
        net.java.games.input.Component x = control.getComponent(Identifier.Button._1);
        return x.getPollData();
    }

    public double getLeftX() {
        control.poll();
        return control.getComponent(Identifier.Axis.X).getPollData();
    }

    public double getLeftY() {
        control.poll();
        return -control.getComponent(Identifier.Axis.Y).getPollData();
    }

    public double getRightY() {
        control.poll();
        return -control.getComponent(Identifier.Axis.RZ).getPollData();
    }

    public double getRightX() {
        control.poll();
        return control.getComponent(Identifier.Axis.Z).getPollData();
    }
}
