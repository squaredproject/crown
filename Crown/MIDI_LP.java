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
  private final Engine engine;
  private final DiscreteParameter automationSlot;
  private final LXAutomationRecorder[] automation;
  private final BooleanParameter[] automationStop;

  final BasicParameter drumpadVelocity;

  private Launchpad launchpad;
  private LaunchpadClient client;

  public class LPListener extends LaunchpadListenerAdapter {

    private final LaunchpadClient client;

    public LPListener(LaunchpadClient client) {
      System.out.println(" created Launchpad Listener ");
      this.client = client;
    }

    // NOTE:
    // Going to trigger on a 9x9 array,
    // so map the top buttons to row 1, and the side buttons to column 9
    // X is the column
    // Y is the row

    @Override
    public void onPadPressed(Pad pad, long timestamp) {
      int x = pad.getX();
      int y = pad.getY();        
        client.setPadLight(pad, Color.RED, BackBufferOperation.NONE);
        ///change colour by Laura 

        if (engine.midiEventTask != null) {
          MIDIEventTask.MIDIEvent e = new MIDIEventTask.MIDIEvent(1, y+1, x, drumpadVelocity.getValuef(), timestamp);
          engine.midiEventTask.add( e ); 
        }

        //drumpad.padTriggered(y+1, x, drumpadVelocity.getValuef() );

    }

    @Override
    public void onPadReleased(Pad pad, long timestamp) {
      int x = pad.getX();
      int y = pad.getY();
      client.setPadLight(pad, Color.YELLOW, BackBufferOperation.NONE);

      if (engine.midiEventTask!= null) {
        MIDIEventTask.MIDIEvent e = new MIDIEventTask.MIDIEvent(2,  y+1, x, drumpadVelocity.getValuef(), timestamp);
        engine.midiEventTask.add( e ); 
      }

      //drumpad.padReleased(y+1, x);

    }

    // Buttons are: 1 through 8 are returning 0 to 7 
    // A through H are numbers 0 to 7
    @Override
    public void onButtonPressed(Button button, long timestamp) {
      int x;
      int y;
      if (button.isTopButton()) {
        x = 0;
        y = button.getCoordinate();
      }
      else {
        x = button.getCoordinate() + 1;
        y = 8;
      }

      if (engine.midiEventTask != null) {
          MIDIEventTask.MIDIEvent e = new MIDIEventTask.MIDIEvent(1, x, y,drumpadVelocity.getValuef(), timestamp);
          engine.midiEventTask.add( e ); 
        }
      
      client.setButtonLight(button, Color.RED, BackBufferOperation.NONE);

    }

    @Override
    public void onButtonReleased(Button button, long timestamp) {
      int x;
      int y;
      if (button.isTopButton()) {
        x = 0;
        y = button.getCoordinate();
      }
      else {
        x = button.getCoordinate() + 1;
        y = 8;
      }

      if (engine.midiEventTask!= null) {
        MIDIEventTask.MIDIEvent e = new MIDIEventTask.MIDIEvent(2, x, y, drumpadVelocity.getValuef(), timestamp);
        engine.midiEventTask.add( e ); 
      }

      client.setButtonLight(button, Color.BLACK, BackBufferOperation.NONE);
      //change colour by Laura
    }


  };

  
  public MidiEngine_LP(final LX lx, final Engine engine, LXListenableNormalizedParameter[] effectKnobParameters, 
        final BasicParameter drumpadVelocity, 
        final BooleanParameter[] previewChannels, final BPMTool bpmTool, final InterfaceController uiDeck, 
        final BasicParameter outputBrightness, DiscreteParameter automationSlot, 
        LXAutomationRecorder[] automation, BooleanParameter[] automationStop) {

    this.lx = lx;
    this.engine = engine;
    this.automationSlot = automationSlot;
    this.automation = automation;
    this.automationStop = automationStop;

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
