import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class SocketComm implements Runnable {
    private ServerSocket sock;
    private Socket client;
    private DataOutputStream writer;
    private DataInputStream reader;
    private ByteBuffer buffer = ByteBuffer.allocate(16);
    public RecievedData rd;
    public SendData sd;
    public Thread TransferThread;
    private boolean isStopped = false;

    public SocketComm() {
        try {
            sock = new ServerSocket(3000);
            System.out.println("Socket: Connecting");
            client = sock.accept();
            System.out.println("Socket: Connected");
            writer = new DataOutputStream(client.getOutputStream());
            reader = new DataInputStream(client.getInputStream());
            buffer.order(ByteOrder.LITTLE_ENDIAN);
        } catch (IOException e) {
            e.printStackTrace();
        }
        rd = new RecievedData();
        sd = new SendData();

        TransferThread = new Thread(this);
        TransferThread.start();
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
            public void run() {
                try {
                    sock.close();
                    client.close();
                } catch (IOException e) {

                }

            }
        }));
        System.out.println("Socket: Ready");
    }

    public void transferData() {
        try {
            byte[] bytes = sd.toByteArray();
            writer.write(bytes);
            writer.flush();
            byte[] incomingData = new byte[16];
            reader.read(incomingData);
            buffer.put(incomingData);
            buffer.position(0);
            rd.roll = buffer.getFloat() * 180 / 3.14;
            rd.pitch = buffer.getFloat() * 180 / 3.14;
            rd.yaw = buffer.getFloat() * 180 / 3.14;
            rd.isStopping = buffer.getFloat();
            buffer.clear();
            rd.lastRecievedData = System.currentTimeMillis();
            Thread.sleep(30);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(0);
        }
    }

    public void stop() {
        isStopped = true;
        try {
            TransferThread.join(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        while (!isStopped) {
            transferData();
        }
    }

    public static byte[] floatArray2ByteArray(float[] values) {
        ByteBuffer buffer = ByteBuffer.allocate(4 * values.length);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        for (float value : values) {
            buffer.putFloat(value);
        }
        return buffer.array();
    }

    class SendData {
        double throttle;
        double rotate;
        double roll;
        double pitch;
        double kill;
        long lastSentData;

        public byte[] toByteArray() {
            lastSentData = System.currentTimeMillis();
            float data[] = new float[]{
                    (float) throttle,
                    (float) rotate,
                    (float) roll,
                    (float) pitch,
                    (float) kill
            };
            return floatArray2ByteArray(data);
        }

    }

    class RecievedData {
        double roll;
        double pitch;
        double yaw;
        long lastRecievedData;
        double isStopping;
    }
}
