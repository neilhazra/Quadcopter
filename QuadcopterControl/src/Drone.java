public class Drone {
    public static JoystickWrapper jw;
    public static SocketComm sc;
    private static double pitchOffset = 0;
    private static double rollOffset = 0;

    public Drone() {
        jw = new JoystickWrapper();
        sc = new SocketComm();
    }

    public static void main(String args[]) throws InterruptedException {
        new Drone();
        long prevLastRecievedData = 0;
        while (Drone.jw.getX() == 0) {
            Drone.periodic();
            long currentReceivedData = Drone.sc.rd.lastRecievedData;
            if (prevLastRecievedData != currentReceivedData) {
                System.out.println(Drone.sc.rd.roll + " " + Drone.sc.rd.pitch + " " + Drone.sc.rd.yaw);
                prevLastRecievedData = currentReceivedData;
            }
            Thread.sleep(20);
        }
        Drone.close();
        while (Drone.sc.rd.isStopping != 1) ;
        System.out.println(Drone.sc.rd.isStopping);
        Drone.sc.stop();
        System.exit(0);
    }

    public static void close() {
        Drone.sc.sd.kill = 1;
    }

    public static void periodic() {
        Drone.sc.sd.throttle = Drone.jw.getLeftY();
        Drone.sc.sd.rotate = Drone.jw.getLeftX();
        double pitch = Drone.jw.getRightY();
        double roll = Drone.jw.getRightX();
        Drone.sc.sd.pitch = pitch + pitchOffset;
        Drone.sc.sd.roll = roll + rollOffset;
        if (Drone.jw.getY() == 1.0) {
            System.out.println("Updating Offsets");
            pitchOffset = pitch;
            rollOffset = roll;
        }
        Drone.sc.sd.kill = Drone.jw.getX();
    }
}
