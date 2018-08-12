import heronarts.lx.LX;
import heronarts.lx.LXAutomationRecorder;
import heronarts.lx.LXChannel;
import heronarts.lx.midi.LXAbstractMidiListener;
import heronarts.lx.midi.LXMidiControlChange;
import heronarts.lx.midi.LXMidiDevice;
import heronarts.lx.midi.LXMidiInput;
import heronarts.lx.midi.LXMidiNoteOff;
import heronarts.lx.midi.LXMidiNoteOn;
import heronarts.lx.midi.LXMidiOutput;
import heronarts.lx.midi.device.APC40;
import heronarts.lx.output.LXDatagramOutput;
import heronarts.lx.parameter.BasicParameter;
import heronarts.lx.parameter.BooleanParameter;
import heronarts.lx.parameter.DiscreteParameter;
import heronarts.lx.parameter.LXListenableNormalizedParameter;
import heronarts.lx.parameter.LXParameter;
import heronarts.lx.parameter.LXParameterListener;

import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiDevice;
import javax.sound.midi.SysexMessage;
import javax.sound.midi.Receiver;
import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiUnavailableException;

//launchpad jars
import net.thecodersbreakfast.lp4j.api.*;
import net.thecodersbreakfast.lp4j.midi.*;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiUnavailableException;



class MidiEngine_LP {

  private final LX lx;
  private final DiscreteParameter automationSlot;
  private final LXAutomationRecorder[] automation;
  private final BooleanParameter[] automationStop;

  final TSDrumpad drumpad;
  final BasicParameter drumpadVelocity;

  private Launchpad launchpad;
  private LaunchpadClient client;

  public class LPListener extends LaunchpadListenerAdapter {

    private final LaunchpadClient client;

    public LPListener(LaunchpadClient client) {
      System.out.println(" created Launchpad Listener ");
      this.client = client;
    }

    @Override
    public void onPadPressed(Pad pad, long timestamp) {
      int x = pad.getX();
      int y = pad.getY();
        System.out.printf(" onPadPressed X %d Y %d velocity %f\n",x,y,drumpadVelocity.getValuef() );
        client.setPadLight(pad, Color.YELLOW, BackBufferOperation.NONE);

        drumpad.padTriggered(x, y, drumpadVelocity.getValuef() );

    }

    @Override
    public void onPadReleased(Pad pad, long timestamp) {
      int x = pad.getX();
      int y = pad.getY();
      System.out.printf(" onPadReleased X %d Y %d velocity %f\n",pad.getX(), pad.getY(),drumpadVelocity.getValuef() );
      client.setPadLight(pad, Color.BLACK, BackBufferOperation.NONE);

      drumpad.padReleased(x, y);

    }

    @Override
    public void onButtonReleased(Button button, long timestamp) {
      System.out.printf(" onButtonReleased coord %d istop %d isright %d\n",button.getCoordinate(), button.isTopButton() ? 1 : 0, button.isRightButton() ? 1 : 0);
      client.setButtonLight(button, Color.BLACK, BackBufferOperation.NONE);
    }


  };

  
  public MidiEngine_LP(final LX lx, LXListenableNormalizedParameter[] effectKnobParameters, 
  			final TSDrumpad drumpad, final BasicParameter drumpadVelocity, 
  			final BooleanParameter[] previewChannels, final BPMTool bpmTool, final InterfaceController uiDeck, 
  			final BasicParameter outputBrightness, DiscreteParameter automationSlot, 
  			LXAutomationRecorder[] automation, BooleanParameter[] automationStop) {

    this.lx = lx;
    this.automationSlot = automationSlot;
    this.automation = automation;
    this.automationStop = automationStop;

    this.drumpad = drumpad;
    this.drumpadVelocity = drumpadVelocity;

    // Init the Midi driver, fail if none
    try {
      this.launchpad = new MidiLaunchpad(MidiDeviceConfiguration.autodetect());
      this.client = launchpad.getClient();
      launchpad.setListener( new LPListener( this.client ));
    }
    catch (MidiUnavailableException e) {
      System.out.println(" Could not init Launchpad, not connected? "+e.toString() );
    }
    catch (net.thecodersbreakfast.lp4j.api.LaunchpadException e) {
      System.out.println(" No Launchpad Found "+e.toString() );
    }


  };
  
}
