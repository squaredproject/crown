import net.thecodersbreakfast.lp4j.api.*;
import net.thecodersbreakfast.lp4j.midi.*;
import java.util.concurrent.CountDownLatch;

import javax.sound.midi.InvalidMidiDataException;


public class HelloWorld {

    private static CountDownLatch stop = new CountDownLatch(1);

    public static void main(String[] args) throws Exception {

        Launchpad launchpad = new MidiLaunchpad(MidiDeviceConfiguration.autodetect());
        LaunchpadClient client = launchpad.getClient();

        MyListener myListener = new MyListener(client);
        launchpad.setListener(myListener);

        // Set a red light under the STOP button
        client.reset();
        client.setButtonLight(Button.STOP, Color.RED, BackBufferOperation.NONE);

        stop.await();
        client.reset();
        launchpad.close();
    }

    public static class MyListener extends LaunchpadListenerAdapter {

        private final LaunchpadClient client;

        public MyListener(LaunchpadClient client) {
            this.client = client;
        }

        @Override
        public void onPadPressed(Pad pad, long timestamp) {
            client.setPadLight(pad, Color.YELLOW, BackBufferOperation.NONE);
        }

        @Override
        public void onPadReleased(Pad pad, long timestamp) {
            client.setPadLight(pad, Color.BLACK, BackBufferOperation.NONE);
        }

        @Override
        public void onButtonReleased(Button button, long timestamp) {
            client.setButtonLight(button, Color.BLACK, BackBufferOperation.NONE);
            switch (button) {
                case STOP:
                    stop.countDown();
                    break;
            }
        }
    }

}

