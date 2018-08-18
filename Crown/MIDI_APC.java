import heronarts.lx.LX;
import heronarts.lx.LXAutomationRecorder;
import heronarts.lx.LXChannel;
import heronarts.lx.midi.*;
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



class MidiEngine_APC {
  // This SYSEX puts an APC40 into ableton mode so we can use it
  // this causes craziness because of mac sysex problems
  final static byte[] APC_MODE_SYSEX = {
    (byte) 0xf0, // sysex start
    (byte) 0x47, // manufacturers id
    (byte) 0x00, // device id
    (byte) 0x73, // product model id
    (byte) 0x60, // message
    (byte) 0x00, // bytes MSB
    (byte) 0x04, // bytes LSB
    (byte) 0x42, // ableton mode 2
    (byte) 0x08, // version maj
    (byte) 0x01, // version min
    (byte) 0x01, // version bugfix
    (byte) 0xf7, // sysex end
  };

  private final LX lx;
  private final DiscreteParameter automationSlot;
  private final LXAutomationRecorder[] automation;
  private final BooleanParameter[] automationStop;
  
  public MidiEngine_APC(final LX lx, LXListenableNormalizedParameter[] effectKnobParameters, 
  			final TSDrumpad drumpad, final BasicParameter drumpadVelocity, 
  			final BooleanParameter[] previewChannels, final BPMTool bpmTool, final InterfaceController uiDeck, 
  			final BasicParameter outputBrightness, DiscreteParameter automationSlot, 
  			LXAutomationRecorder[] automation, BooleanParameter[] automationStop) {
    this.lx = lx;
    this.automationSlot = automationSlot;
    this.automation = automation;
    this.automationStop = automationStop;
    try {
      setAPC40Mode();
    } catch (java.lang.UnsatisfiedLinkError e){
      System.out.println("could not set up APC40: unsatisfied link error, caused by mmj on non mac");
      return;
    }
    LXMidiInput apcInput = APC40.matchInput(lx);
    LXMidiOutput apcOutput = APC40.matchOutput(lx);
        
    if (apcInput != null) {
      
      // Add this input to the midi engine so that events are recorded
      lx.engine.midiEngine.addInput(apcInput);
      lx.engine.midiEngine.addListener(new LXAbstractMidiListener() {
        public void noteOnReceived(LXMidiNoteOn note) {
          int channel = note.getChannel();
          int pitch = note.getPitch();
          //System.out.printf(" Got APC40::: noteOffReceived %d %d\n",channel,pitch);

          switch (pitch) {
          case APC40.CLIP_LAUNCH:
          case APC40.CLIP_LAUNCH+1:
          case APC40.CLIP_LAUNCH+2:
          case APC40.CLIP_LAUNCH+3:
          case APC40.CLIP_LAUNCH+4:
            drumpad.padTriggered(pitch - APC40.CLIP_LAUNCH, channel, drumpadVelocity.getValuef());
            break;
          case APC40.CLIP_STOP:
            drumpad.padTriggered(5, channel, drumpadVelocity.getValuef());
            break;
          case APC40.SCENE_LAUNCH:
          case APC40.SCENE_LAUNCH+1:
          case APC40.SCENE_LAUNCH+2:
          case APC40.SCENE_LAUNCH+3:
          case APC40.SCENE_LAUNCH+4:
            drumpad.padTriggered(pitch - APC40.SCENE_LAUNCH, 8, drumpadVelocity.getValuef());
            break;
          case APC40.STOP_ALL_CLIPS:
            drumpad.padTriggered(5, 8, drumpadVelocity.getValuef());
            break;
          }
        }
        
        public void noteOffReceived(LXMidiNote note) {
          //System.out.printf(" Got APC40::: noteOffReceived \n");

          int channel = note.getChannel();
          int pitch = note.getPitch();

          switch (pitch) {
          case APC40.CLIP_LAUNCH:
          case APC40.CLIP_LAUNCH+1:
          case APC40.CLIP_LAUNCH+2:
          case APC40.CLIP_LAUNCH+3:
          case APC40.CLIP_LAUNCH+4:
            drumpad.padReleased(pitch - APC40.CLIP_LAUNCH, channel);
            break;
          case APC40.CLIP_STOP:
            drumpad.padReleased(5, channel);
            break;
          case APC40.SCENE_LAUNCH:
          case APC40.SCENE_LAUNCH+1:
          case APC40.SCENE_LAUNCH+2:
          case APC40.SCENE_LAUNCH+3:
          case APC40.SCENE_LAUNCH+4:
            drumpad.padReleased(pitch - APC40.SCENE_LAUNCH, 8);
            break;
          case APC40.STOP_ALL_CLIPS:
            drumpad.padReleased(5, 8);
            break;
          }
        }
      });

      final APC40 apc40 = new APC40(apcInput, apcOutput) {
        protected void noteOn(LXMidiNoteOn note) {
          int channel = note.getChannel();
          switch (note.getPitch()) {
          
          case APC40.SOLO_CUE:
            if (previewChannels[channel].isOn() && channel != focusedChannel()) {
              lx.engine.focusedChannel.setValue(channel);
            }
            break;
                        
          //case APC40.SEND_A:
          //  bpmTool.beatType.increment();
          //  break;
          //case APC40.SEND_B:
          //  bpmTool.tempoLfoType.increment();
          //  break;
            
          case APC40.MASTER_TRACK:
          case APC40.SHIFT:
            if (uiDeck != null) uiDeck.select();
            break;
          case APC40.BANK_UP:
            if (uiDeck != null) uiDeck.scroll(-1);
            break;
          case APC40.BANK_DOWN:
            if (uiDeck != null) uiDeck.scroll(1);
            break;
          case APC40.BANK_RIGHT:
            lx.engine.focusedChannel.increment();
            break;
          case APC40.BANK_LEFT:
            lx.engine.focusedChannel.decrement();
            break;
          }
        }
        
        protected void controlChange(LXMidiControlChange controller) {
          switch (controller.getCC()) {
          case APC40.CUE_LEVEL:
            if (uiDeck != null) uiDeck.knob(controller.getValue());
            break;
          }
        }
      };

      for (int row = 0; row < drumpad.triggerables.length && row < 6; row++) {
        int midiNumber;
        if (row < 5) {
          midiNumber = APC40.CLIP_LAUNCH + row;
        } else {
          midiNumber = APC40.CLIP_STOP;
        }
        for (int col = 0; col < drumpad.triggerables[row].length && col < 9; col++) {
          if (col < 8) {
            //apc40.bindNote(nfcToggles[row][col], col, midiNumber, APC40.DIRECT);
          } else if (row < 5) {
            //apc40.bindNote(nfcToggles[row][col], 0, APC40.SCENE_LAUNCH + row, APC40.DIRECT);
            // stop all clips button doesn't light up. Doesn't have an LED in it
            // apc40.bindNote(new BooleanParameter("ANON", false), 0, APC40.STOP_ALL_CLIPS, APC40.DIRECT);
          }
        }
      }
   
      int[] channelIndices = new int[Engine.NUM_CHANNELS];
      for (int i = 0; i < Engine.NUM_CHANNELS; ++i) {
        channelIndices[i] = i;
      }
      
      // Track selection
      apc40.bindNotes(lx.engine.focusedChannel, channelIndices, APC40.TRACK_SELECTION);
      
      for (int i = 0; i < Engine.NUM_CHANNELS; i++) {
        // Cue activators
        apc40.bindNote(previewChannels[i], i, APC40.SOLO_CUE, LXMidiDevice.TOGGLE);

        apc40.bindController(lx.engine.getChannel(i).getFader(), i, APC40.VOLUME, LXMidiDevice.TakeoverMode.PICKUP);
      }
      
      for (int i = 0; i < 8; ++i) {
        apc40.sendController(0, APC40.TRACK_CONTROL_LED_MODE + i, APC40.LED_MODE_VOLUME);
        apc40.sendController(0, APC40.DEVICE_CONTROL_LED_MODE + i, APC40.LED_MODE_VOLUME);
      }
      
      // Master fader
      apc40.bindController(outputBrightness, 0, APC40.MASTER_FADER, LXMidiDevice.TakeoverMode.PICKUP);

      apc40.bindController(drumpadVelocity, 0, APC40.CROSSFADER);
      
      // Effect knobs + buttons
      for (int i = 0; i < effectKnobParameters.length; ++i) {
        if (effectKnobParameters[i] != null) {
          apc40.bindController(effectKnobParameters[i], 0, APC40.TRACK_CONTROL + i);
        }
      }
      
      // Pattern control
      apc40.bindDeviceControlKnobs(lx.engine);
      lx.engine.focusedChannel.addListener(new LXParameterListener() {
        public void onParameterChanged(LXParameter parameter) {
          apc40.bindNotes(
            getFaderTransition(lx.engine.getFocusedChannel()).blendMode,
            0,
            new int[] { APC40.CLIP_TRACK, APC40.DEVICE_ON_OFF, APC40.LEFT_ARROW, APC40.RIGHT_ARROW }
          );
        }
      });
      
      // Tap Tempo
      apc40.bindNote(new BooleanParameter("ANON", false), 0, APC40.SEND_A, APC40.DIRECT);
      apc40.bindNote(new BooleanParameter("ANON", false), 0, APC40.SEND_B, APC40.DIRECT);
      //apc40.bindNote(bpmTool.addTempoLfo, 0, APC40.PAN, APC40.DIRECT);
      //apc40.bindNote(bpmTool.clearAllTempoLfos, 0, APC40.SEND_C, APC40.DIRECT);
      //apc40.bindNote(bpmTool.tapTempo, 0, APC40.TAP_TEMPO, APC40.DIRECT);
      //apc40.bindNote(bpmTool.nudgeUpTempo, 0, APC40.NUDGE_PLUS, APC40.DIRECT);
      //apc40.bindNote(bpmTool.nudgeDownTempo, 0, APC40.NUDGE_MINUS, APC40.DIRECT);
      
      apc40.bindNotes(
        getFaderTransition(lx.engine.getFocusedChannel()).blendMode,
        0,
        new int[] { APC40.CLIP_TRACK, APC40.DEVICE_ON_OFF, APC40.LEFT_ARROW, APC40.RIGHT_ARROW }
      );
      apc40.bindNotes(
        automationSlot,
        0,
        new int[] { APC40.DETAIL_VIEW, APC40.REC_QUANTIZATION, APC40.MIDI_OVERDUB, APC40.METRONOME }
      );
      automationSlot.addListener(new LXParameterListener() {
        public void onParameterChanged(LXParameter parameter) {
          setAutomation(apc40);
        }
      });
      setAutomation(apc40);
    }

  }
  
  void setAutomation(APC40 apc40) {
    LXAutomationRecorder auto = automation[automationSlot.getValuei()];
    apc40.bindNoteOn(auto.isRunning, 0, APC40.PLAY, LXMidiDevice.TOGGLE);
    apc40.bindNoteOn(auto.armRecord, 0, APC40.REC, LXMidiDevice.TOGGLE);
    apc40.bindNote(automationStop[automationSlot.getValuei()], 0, APC40.STOP, LXMidiDevice.DIRECT);
  }
  
// This is theproblem with the APC40 that it requires a special SYSEX to say we're
// ableton. The Mac requires using this obsolete library, with windows we don't,
// so we literally have to check the operating system version.
//

  void setAPC40Mode() {

  	boolean sentSysEx = false;
    int i = 0;


    //System.out.println(" APC40 2 Mode --- for non-mac ");
    MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();
    //System.out.println(" length of array is "+infos.length);

    for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo()) {

    	System.out.println(" looking for the APC40 "+info.toString());

    	if (info.toString().contains("APC40")) {

    		System.out.println(" Found APC40 - send sysex");
    		try {
	    		SysexMessage sysMsg = new SysexMessage(  );
	    		sysMsg.setMessage(APC_MODE_SYSEX, APC_MODE_SYSEX.length);

	    		MidiDevice dev = MidiSystem.getMidiDevice(info);
	    		dev.open();
	    		Receiver r = dev.getReceiver();

	    		r.send(sysMsg, -1);
	    		sentSysEx = true;
	    		dev.open();
	    	}
	    	catch ( InvalidMidiDataException e ) {
				System.out.println("InvalidMidiDataException: sysex send " + e.getMessage());
	    	}
	    	catch ( MidiUnavailableException e ) {
				System.out.println("MidiUnavailableException: sysex send " + e.getMessage());
	    	}

        	break;
    	}
    	i++;
    }
  }

 

  int focusedChannel() {
    return lx.engine.focusedChannel.getValuei();
  }

  CrownTransition getFaderTransition(LXChannel channel) {
    return (CrownTransition) channel.getFaderTransition();
  }
}
