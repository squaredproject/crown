import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Reader;
import java.lang.reflect.Type;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;
import java.util.ArrayDeque;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.reflect.TypeToken;

import heronarts.lx.LX;
import heronarts.lx.LXAutomationRecorder;
import heronarts.lx.LXChannel;
import heronarts.lx.LXEngine;
import heronarts.lx.color.LXColor;
import heronarts.lx.effect.BlurEffect;
import heronarts.lx.effect.LXEffect;            
import heronarts.lx.model.LXModel;
import heronarts.lx.LXLoopTask;
import heronarts.lx.output.FadecandyOutput;
import heronarts.lx.output.LXDatagram;
import heronarts.lx.output.LXDatagramOutput;
import heronarts.lx.parameter.BasicParameter;
import heronarts.lx.parameter.BooleanParameter;
import heronarts.lx.parameter.DiscreteParameter;
import heronarts.lx.parameter.LXListenableNormalizedParameter;
import heronarts.lx.parameter.LXParameter;
import heronarts.lx.parameter.LXParameterListener;
import heronarts.lx.pattern.LXPattern;
import heronarts.lx.transition.DissolveTransition;
import heronarts.lx.transition.LXTransition;

abstract class Engine {

  static final int NUM_CHANNELS = 8;
  static final int NUM_KNOBS = 8;
  static final int NUM_AUTOMATION = 4;

  final String projectPath;
  final List<ClusterConfig> clusterConfig;
  final LX lx;
  final Model model;
  EngineController engineController;
  LXDatagramOutput output;
  LXDatagram[] datagrams;
  BPMTool bpmTool;
  InterfaceController uiDeck;
  MidiEngine_APC midiEngine_APC;
  MidiEngine_LP midiEngine_LP;
  TSDrumpad drumpad_APC;
  TSDrumpad drumpad_LP;
  public MIDIEventTask midiEventTask; // transition between outside context and LX
  LXListenableNormalizedParameter[] effectKnobParameters;
  final BasicParameter dissolveTime = new BasicParameter("DSLV", 400, 50, 1000);
  final BasicParameter drumpadVelocity = new BasicParameter("DVEL", 1);

  // these exist both here and in Crown.pde, and need to be shared.
  // these are the ones that get created, the UI reaches over here to grab them.
  final TSAutomationRecorder[] automation = new TSAutomationRecorder[Engine.NUM_AUTOMATION];
  final BooleanParameter[] automationStop = new BooleanParameter[Engine.NUM_AUTOMATION]; 
  final DiscreteParameter automationSlot = new DiscreteParameter("AUTO", Engine.NUM_AUTOMATION);
  final BooleanParameter[] previewChannels = new BooleanParameter[Engine.NUM_CHANNELS];


  final BasicParameterProxy outputBrightness = new BasicParameterProxy(1);
  final BrightnessScaleEffect masterBrightnessEffect;

  Engine(String projectPath) {
    this.projectPath = projectPath;

    clusterConfig = loadConfigFile(Config.CLUSTER_CONFIG_FILE);
    model = new Model(clusterConfig, projectPath + "/data/" );
    lx = createLX();
    engineController = new EngineController(lx);
    masterBrightnessEffect = new BrightnessScaleEffect(lx);
  
    lx.engine.addParameter(drumpadVelocity);

    if (Config.enableAPC40) {
      drumpad_APC = new TSDrumpad();
    }

    if (Config.enableLaunchpad ) {
      drumpad_LP = new TSDrumpad();
    }

    // there is probably a better place for this, but I don't know where
    // needs doing before the UI configures lots of bits
    for (int i=0; i < Engine.NUM_CHANNELS; i++ ) {
      previewChannels[i] = new BooleanParameter("PRV");
    }

    configureChannels();
    configureTriggerables();
    configureBMPTool();
    configureAutomation();

    if (drumpad_APC != null) {
      System.out.println(" setting drumpad APC triggerables ");
      drumpad_APC.triggerables = drumpadTriggerables_APC;
    }
    if (drumpad_LP != null) {
      System.out.println(" setting drumpad LP triggerables ");
      drumpad_LP.triggerables = drumpadTriggerables_LP;
    }

    // ordering: these create the 'drumpad' object if neccesary,
    // must be before registration of the triggerables
    if (Config.enableAPC40) {
      configureMIDI_APC40();
    }
    if (Config.enableLaunchpad) {
      configureMIDI_Launchpad();
    }

    lx.engine.addLoopTask(new ModelTransformTask(model));

    if (drumpad_LP != null) {
      midiEventTask = new MIDIEventTask(drumpad_LP);
      lx.engine.addLoopTask(midiEventTask);
    }

    if (Config.enableOutputBigtree) {
      System.out.println("NDB output enabled");
      lx.addEffect(new TurnOffDeadPixelsEffect(lx));
      configureExternalOutput();
    }

    lx.addEffect(masterBrightnessEffect);

    configureEffects();

    postCreateLX();

    
    // bad code I know
    // (shouldn't mess with engine internals)
    // maybe need a way to specify a deck shouldn't be focused?
    // essentially this lets us have extra decks for the drumpad
    // patterns without letting them be assigned to channels
    // -kf
    lx.engine.focusedChannel.setRange(Engine.NUM_CHANNELS);
  }

  void start() {
    lx.engine.start();
  }

  abstract LX createLX();

  void postCreateLX() { }

  void addPatterns(ArrayList<LXPattern> patterns) {
    // Add patterns here.
    // The order here is the order it shows up in the patterns list
    // patterns.add(new SolidColor(lx));
    // patterns.add(new ClusterLineTest(lx));
    // patterns.add(new OrderTest(lx));
    patterns.add(new Twister(lx));
    patterns.add(new CandyCloud(lx));
    patterns.add(new MarkLottor(lx));
    patterns.add(new SolidColor(lx));
    // patterns.add(new DoubleHelix(lx)); // removed because cluster sensitive
    patterns.add(new SparkleHelix(lx));
    patterns.add(new Lightning(lx));
    patterns.add(new SparkleTakeOver(lx));
    patterns.add(new MultiSine(lx));
    patterns.add(new Ripple(lx));
    patterns.add(new SeeSaw(lx));
    //patterns.add(new SweepPattern(lx)); // removed because is cluster sensitive
    patterns.add(new IceCrystals(lx));
    patterns.add(new ColoredLeaves(lx));
    patterns.add(new Stripes(lx));
    patterns.add(new AcidTrip(lx));
    patterns.add(new Springs(lx));
    patterns.add(new Lattice(lx));
    patterns.add(new Fire(lx));
    patterns.add(new Fireflies(lx));
    patterns.add(new Fumes(lx));
    patterns.add(new Voronoi(lx));
    patterns.add(new Cells(lx));
    patterns.add(new Bubbles(lx));
    patterns.add(new Pulleys(lx));

    patterns.add(new Wisps(lx));
    patterns.add(new Explosions(lx));
    patterns.add(new BassSlam(lx));
    patterns.add(new Rain(lx));
    patterns.add(new Fade(lx));
    patterns.add(new Strobe(lx));
    patterns.add(new Twinkle(lx));
    patterns.add(new VerticalSweep(lx));
    patterns.add(new RandomColor(lx));
    patterns.add(new ColorStrobe(lx));
    patterns.add(new Pixels(lx));
    patterns.add(new Wedges(lx));
    patterns.add(new Parallax(lx));
    patterns.add(new RotateTower(lx));
    patterns.add(new ThrobTower(lx));
    patterns.add(new RingularityPattern(lx));
    
    patterns.add(new T1Pattern(lx));
    patterns.add(new T2Pattern(lx));
  }

  LXPattern[] getPatternListForChannels() {
    ArrayList<LXPattern> patterns = new ArrayList<LXPattern>();
    addPatterns(patterns);
    for (LXPattern pattern : patterns) {
      LXTransition t = new DissolveTransition(lx).setDuration(dissolveTime);
      pattern.setTransition(t);
    }
    return patterns.toArray(new LXPattern[patterns.size()]);
  }

  void registerPatternTriggerables() {

    // The 2rd parameter is which row of the drumpad to add it to.
    // defaults to the 3rd row
    // the row parameter is zero indexed
    registerPattern(new Twister(lx));
    registerPattern(new MarkLottor(lx));
    registerPattern(new Ripple(lx));
    registerPattern(new Stripes(lx));
    registerPattern(new Lattice(lx));
    registerPattern(new Fumes(lx));
    registerPattern(new Voronoi(lx));
    registerPattern(new CandyCloud(lx));
    registerPattern(new GalaxyCloud(lx));

    registerPattern(new ColorStrobe(lx), 3);
    registerPattern(new Explosions(lx, 20),  3);
    registerPattern(new Strobe(lx),  3);
    registerPattern(new SparkleTakeOver(lx), 3);
    registerPattern(new MultiSine(lx), 3);
    registerPattern(new SeeSaw(lx), 3);
    registerPattern(new Cells(lx), 3);
    registerPattern(new Fade(lx), 3);
    registerPattern(new Pixels(lx), 3);
    
    registerPattern(new IceCrystals(lx), 5);
    registerPattern(new Fire(lx), 5); // Make red
    
    registerPattern(new AcidTrip(lx));
    registerPattern(new Rain(lx));

    registerPattern(new Wisps(lx, 1, 60, 50, 270, 20, 3.5, 10)); // downward yellow wisp
    registerPattern(new Wisps(lx, 30, 210, 100, 90, 20, 3.5, 10)); // colorful wisp storm
    registerPattern(new Wisps(lx, 1, 210, 100, 90, 130, 3.5, 10)); // multidirection colorful wisps
    registerPattern(new Wisps(lx, 3, 210, 10, 270, 0, 3.5, 10)); // rain storm of wisps
    registerPattern(new Wisps(lx, 35, 210, 180, 180, 15, 2, 15)); // twister of wisps
  }

  // This subsumes the old "PatternTriggerables" and the old "OneShot"
  void registerPatternTriggerables_LP() {

    ArrayList<Triggerable>[] pad = drumpadTriggerablesLists_LP;
    // The 2rd parameter is which row of the drumpad to add it to.
    // defaults to the 3rd row
    // the row parameter is zero indexed

    // for initial attempt, put every element to 
    // same really visual output


    // first row - random strobe color?  
    for (int i=0;i<drumpadColumns_LP;i++) {
      registerVisualDrumpad(new ColorStrobe(lx), pad[0]);
    }

    // Second row - particular color strobe
    registerVisualDrumpad(new ColorStrobeColor(lx,0 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,1 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,2 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,3 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,4 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,5 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,6 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,7 * 40), pad[1]);
    registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[1]);

    // third row --- these don't really have effects, seems like they make a pattern current???
    //registerVisualDrumpad(new Twister(lx), pad[2]);
    //registerVisualDrumpad(new SeeSaw(lx), pad[2]);
    //registerVisualDrumpad(new MarkLottor(lx),pad[2]);
    //registerVisualDrumpad(new Ripple(lx),pad[2]);
    //registerVisualDrumpad(new Stripes(lx),pad[2]);
    //registerVisualDrumpad(new Lattice(lx),pad[2]);
    //registerVisualDrumpad(new Fumes(lx),pad[2]);
    //registerVisualDrumpad(new Voronoi(lx),pad[2]);
    //registerVisualDrumpad(new CandyCloud(lx),pad[2]);
    //registerVisualDrumpad(new GalaxyCloud(lx),pad[2]);
    registerVisualDrumpad(new Pulleys(lx), pad[2]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[2]);
    registerVisualDrumpad(new BassSlam(lx), pad[2]);
    registerVisualDrumpad(new Fireflies(lx, 70, 6, 180), pad[2]);
    registerVisualDrumpad(new Fireflies(lx, 40, 7.5f, 90), pad[2]);
    registerVisualDrumpad(new Pulleys(lx), pad[2]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[2]);
    registerVisualDrumpad(new BassSlam(lx), pad[2]);
    registerVisualDrumpad(new Fireflies(lx), pad[2]);
    
    // fourth row
    registerVisualDrumpad(new Explosions(lx, 20),  pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,0 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,1 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,2 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,3 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,4 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,5 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,6 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,7 * 40), pad[3]);
    registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[3]);
    //registerVisualDrumpad(new Explosions(lx, 20),  pad[3]);
    //registerVisualDrumpad(new Strobe(lx),  pad[3]);
    //registerVisualDrumpad(new Wisps(lx, 1, 60, 50, 270, 20, 3.5, 10),pad[3]); // downward yellow wisp
    //registerVisualDrumpad(new Wisps(lx, 30, 210, 100, 90, 20, 3.5, 10),pad[3]); // colorful wisp storm
    //registerVisualDrumpad(new Wisps(lx, 1, 210, 100, 90, 130, 3.5, 10),pad[3]); // multidirection colorful wisps
    //registerVisualDrumpad(new Wisps(lx, 3, 210, 10, 270, 0, 3.5, 10),pad[3]); // rain storm of wisps
    //registerVisualDrumpad(new Wisps(lx, 35, 210, 180, 180, 15, 2, 15),pad[3]); // twister of wisps
    //registerVisualDrumpad(new IceCrystals(lx), pad[3]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,4 * 40), pad[3]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,5 * 40), pad[3]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,6 * 40), pad[3]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,7 * 40), pad[3]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[3]);
    //registerVisualDrumpad(new Rain(lx),pad[3]);
    //registerVisualDrumpad(new SparkleTakeOver(lx), pad[3]);
    //registerVisualDrumpad(new MultiSine(lx), pad[3]);
    //registerVisualDrumpad(new SeeSaw(lx), pad[3]);
    //registerVisualDrumpad(new Cells(lx), pad[3]);
    //registerVisualDrumpad(new Fade(lx), pad[3]);
    //registerVisualDrumpad(new Pixels(lx), pad[3]);

    // 5th row
    registerVisualDrumpad(new Pulleys(lx), pad[4]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[4]);
    registerVisualDrumpad(new BassSlam(lx), pad[4]);
    registerVisualDrumpad(new Fireflies(lx, 70, 6, 180), pad[4]);
    registerVisualDrumpad(new Fireflies(lx, 40, 7.5f, 90), pad[4]);
    registerVisualDrumpad(new Pulleys(lx), pad[4]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[4]);
    registerVisualDrumpad(new BassSlam(lx), pad[4]);
    registerVisualDrumpad(new Fireflies(lx), pad[4]);
    //registerVisualDrumpad(new Explosions(lx), pad[4]);

    // 6th row
    //registerVisualDrumpad(new Fireflies(lx), pad[5]);
    //registerVisualDrumpad(new Bubbles(lx), pad[5]);
    //registerVisualDrumpad(new Lightning(lx), pad[5]);
    //registerVisualDrumpad(new Wisps(lx), pad[5]);
    //registerVisualDrumpad(new ColorStrobeColor(lx,0 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,1 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,2 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,3 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,4 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,5 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,6 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,7 * 40), pad[5]);
    registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[5]);
    registerVisualDrumpad(new Explosions(lx), pad[5]);
    
    // 7th row
    registerVisualDrumpad(new Fireflies(lx), pad[6]);
    registerVisualDrumpad(new Bubbles(lx), pad[6]);
    registerVisualDrumpad(new Lightning(lx), pad[6]);
    //registerVisualDrumpad(new IceCrystals(lx), pad[6]);
    registerVisualDrumpad(new Fire(lx), pad[6]); // Make red
    //registerVisualDrumpad(new AcidTrip(lx),pad[6]);
    registerVisualDrumpad(new Rain(lx),pad[6]);
    registerVisualDrumpad(new ColorStrobeColor(lx,0 * 40), pad[6]);
    registerVisualDrumpad(new ColorStrobeColor(lx,1 * 40), pad[6]);
    registerVisualDrumpad(new ColorStrobeColor(lx,2 * 40), pad[6]);
    registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[6]);

    // 8th row
    //registerVisualDrumpad(new Wisps(lx, 1, 60, 50, 270, 20, 3.5, 10),pad[7]); // downward yellow wisp
    //registerVisualDrumpad(new Wisps(lx, 30, 210, 100, 90, 20, 3.5, 10),pad[7]); // colorful wisp storm
    //registerVisualDrumpad(new Wisps(lx, 1, 210, 100, 90, 130, 3.5, 10),pad[7]); // multidirection colorful wisps
    //registerVisualDrumpad(new Wisps(lx, 3, 210, 10, 270, 0, 3.5, 10),pad[7]); // rain storm of wisps
    //registerVisualDrumpad(new Wisps(lx, 35, 210, 180, 180, 15, 2, 15),pad[7]); // twister of wisps
    registerVisualDrumpad(new Fireflies(lx), pad[7]);
    registerVisualDrumpad(new Bubbles(lx), pad[7]);
    registerVisualDrumpad(new Lightning(lx), pad[7]);
    registerVisualDrumpad(new Pulleys(lx), pad[7]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[7]);
    registerVisualDrumpad(new IceCrystals(lx), pad[7]);
    registerVisualDrumpad(new Fire(lx), pad[7]); // Make red
    registerVisualDrumpad(new Pulleys(lx), pad[7]);
    registerVisualDrumpad(new BassSlam(lx), pad[7]);


    // 9th row
    //registerVisualDrumpad(new Wisps(lx, 1, 60, 50, 270, 20, 3.5, 10),pad[8]); // downward yellow wisp
    //registerVisualDrumpad(new Wisps(lx, 30, 210, 100, 90, 20, 3.5, 10),pad[8]); // colorful wisp storm
    //registerVisualDrumpad(new Wisps(lx, 1, 210, 100, 90, 130, 3.5, 10),pad[8]); // multidirection colorful wisps
    //registerVisualDrumpad(new Wisps(lx, 3, 210, 10, 270, 0, 3.5, 10),pad[8]); // rain storm of wisps
    //registerVisualDrumpad(new Wisps(lx, 35, 210, 180, 180, 15, 2, 15),pad[8]); // twister of wisps
    registerVisualDrumpad(new Pulleys(lx), pad[8]);
    registerVisualDrumpad(new StrobeOneshot(lx), pad[8]);
    registerVisualDrumpad(new BassSlam(lx), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,5 * 40), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,6 * 40), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,7 * 40), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,8 * 40), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,4 * 40), pad[8]);
    registerVisualDrumpad(new ColorStrobeColor(lx,3 * 40), pad[8]);
  }


  void registerOneShotTriggerables() {
    registerOneShot(new Pulleys(lx));
    registerOneShot(new StrobeOneshot(lx));
    registerOneShot(new BassSlam(lx));
    registerOneShot(new Fireflies(lx, 70, 6, 180));
    registerOneShot(new Fireflies(lx, 40, 7.5f, 90));

    registerOneShot(new Fireflies(lx), 5);
    registerOneShot(new Bubbles(lx), 5);
    registerOneShot(new Lightning(lx), 5);
    registerOneShot(new Wisps(lx), 5);
    registerOneShot(new Explosions(lx), 5);
  }

  void registerEffectTriggerables() {
    BlurEffect blurEffect = new TSBlurEffect(lx);
    ColorEffect colorEffect = new ColorEffect(lx);
    GhostEffect ghostEffect = new GhostEffect(lx);
    ScrambleEffect scrambleEffect = new ScrambleEffect(lx);
    StaticEffect staticEffect = new StaticEffect(lx);
    RotationEffect rotationEffect = new RotationEffect(lx);
    SpinEffect spinEffect = new SpinEffect(lx);
    SpeedEffect speedEffect = new SpeedEffect(lx);
    ColorStrobeTextureEffect colorStrobeTextureEffect = new ColorStrobeTextureEffect(lx);
    FadeTextureEffect fadeTextureEffect = new FadeTextureEffect(lx);
    AcidTripTextureEffect acidTripTextureEffect = new AcidTripTextureEffect(lx);
    CandyTextureEffect candyTextureEffect = new CandyTextureEffect(lx);
    CandyCloudTextureEffect candyCloudTextureEffect = new CandyCloudTextureEffect(lx);

    lx.addEffect(blurEffect);
    lx.addEffect(colorEffect);
    lx.addEffect(ghostEffect);
    lx.addEffect(scrambleEffect);
    lx.addEffect(staticEffect);
    lx.addEffect(rotationEffect);
    lx.addEffect(spinEffect);
    lx.addEffect(speedEffect);
    lx.addEffect(colorStrobeTextureEffect);
    lx.addEffect(fadeTextureEffect);
    lx.addEffect(acidTripTextureEffect);
    lx.addEffect(candyTextureEffect);
    lx.addEffect(candyCloudTextureEffect);

    registerEffectControlParameter(speedEffect.speed, 1, 0.4);
    registerEffectControlParameter(speedEffect.speed, 1, 5);
    registerEffectControlParameter(colorEffect.rainbow);
    registerEffectControlParameter(colorEffect.mono);
    registerEffectControlParameter(colorEffect.desaturation);
    registerEffectControlParameter(colorEffect.sharp);
    registerEffectControlParameter(blurEffect.amount, 0.65);
    registerEffectControlParameter(spinEffect.spin, 0.65);
    registerEffectControlParameter(ghostEffect.amount, 0, 0.16, 1);
    registerEffectControlParameter(scrambleEffect.amount, 0, 1, 1);
    registerEffectControlParameter(colorStrobeTextureEffect.amount, 0, 1, 1);
    registerEffectControlParameter(fadeTextureEffect.amount, 0, 1, 1);
    registerEffectControlParameter(acidTripTextureEffect.amount, 0, 1, 1);
    registerEffectControlParameter(candyCloudTextureEffect.amount, 0, 1, 1);
    registerEffectControlParameter(staticEffect.amount, 0, .3, 1);
    registerEffectControlParameter(candyTextureEffect.amount, 0, 1, 5);

    effectKnobParameters = new LXListenableNormalizedParameter[] {
      colorEffect.hueShift,
      colorEffect.mono,
      colorEffect.desaturation,
      colorEffect.sharp,
      blurEffect.amount,
      speedEffect.speed,
      spinEffect.spin,
      candyCloudTextureEffect.amount
    };
  }

  VisualType[] readerPatternTypeRestrictions() {
    return new VisualType[] {
      VisualType.Pattern,
      VisualType.Pattern,
      VisualType.Pattern,
      VisualType.OneShot,
      VisualType.OneShot,
      VisualType.OneShot,
      VisualType.Effect,
      VisualType.Effect,
      VisualType.Effect,
      VisualType.Pattern,
    };
  }

  String sketchPath(String filename) {
    return projectPath + "/" + filename;
  }
  
  String dataPath(String filename) {
    return projectPath + "/data/" + filename;
  }

  List<ClusterConfig> loadConfigFile(String filename) {
    return loadJSONFile(filename, new TypeToken<List<ClusterConfig>>() {}.getType());
  }

  JsonArray loadSavedSetFile(String filename) {
    return loadJSONFile(filename, JsonArray.class);
  }

  <T> T loadJSONFile(String filename, Type typeToken) {
    Reader reader = null;
    try {
      reader = new BufferedReader(new FileReader(sketchPath(filename)));
      return new Gson().fromJson(reader, typeToken);
    } catch (IOException ioe) { 
      System.out.println("Error reading json file: ");
      System.out.println(ioe);
    } finally {
      if (reader != null) {
        try {
          reader.close();
        } catch (IOException ioe) { }
      }
    }
    return null;
  }

  void saveJSONToFile(List<ClusterConfig> config, String filename) {
    PrintWriter writer = null;
    try {
      writer = new PrintWriter(new BufferedWriter(new FileWriter(sketchPath(filename))));
      writer.write(new Gson().toJson(config));
    } catch (IOException ioe) {
      System.out.println("Error writing json file.");
    } finally {
      if (writer != null) {
        writer.close();
      }
    }
  }

  /* configureChannels */

  void setupChannel(final LXChannel channel, boolean noOpWhenNotRunning) {

    channel.setFaderTransition(new CrownTransition(lx, channel));

    channel.addListener(new LXChannel.AbstractListener() {
      LXTransition transition;

      public void patternWillChange(LXChannel channel, LXPattern pattern, LXPattern nextPattern) {
        if (!channel.enabled.isOn()) {
          transition = nextPattern.getTransition();
          nextPattern.setTransition(null);
        }
      }

      public void patternDidChange(LXChannel channel, LXPattern pattern) {
        if (transition != null) {
          pattern.setTransition(transition);
          transition = null;
        }
      }
    });

    if (noOpWhenNotRunning) {
      channel.enabled.setValue(channel.getFader().getValue() != 0);
      channel.getFader().addListener(new LXParameterListener() {
        public void onParameterChanged(LXParameter parameter) {
          channel.enabled.setValue(channel.getFader().getValue() != 0);
        }
      });
    }
  }

  ArrayList<TSPattern> patterns;

  void configureChannels() {
    for (int i = 0; i < Engine.NUM_CHANNELS; ++i) {

      LXChannel channel = lx.engine.addChannel(getPatternListForChannels());
      setupChannel(channel, true);
      if (i == 0) {
        channel.getFader().setValue(1);
      }
      channel.goIndex(i);
    }
    engineController.baseChannelIndex = lx.engine.getChannels().size() - 1;

    lx.engine.removeChannel(lx.engine.getDefaultChannel());
  }

  void registerOneShot(TSPattern pattern) {
    registerOneShot(pattern, 4);
  }

  void registerOneShot(TSPattern pattern, int drumpadRow) {
    registerVisual(pattern, drumpadRow, VisualType.OneShot);
  }

  void registerPattern(TSPattern pattern) {
    registerPattern(pattern, 2);
  }

  void registerPattern(TSPattern pattern, int drumpadRow) {
    registerVisual(pattern, drumpadRow, VisualType.Pattern);
  }

  void registerVisual(TSPattern pattern, int drumpadRow, VisualType visualType) {
    LXTransition t = new DissolveTransition(lx).setDuration(dissolveTime);
    pattern.setTransition(t);

    Triggerable triggerable = configurePatternAsTriggerable(pattern);
    BooleanParameter toggle = null;
    if (drumpad_APC != null) {
      drumpadTriggerablesLists_APC[drumpadRow].add(triggerable);
    }
  }

  void registerVisualDrumpad(TSPattern pattern, ArrayList<Triggerable> drumpadRow) {
    LXTransition t = new DissolveTransition(lx).setDuration(dissolveTime);
    pattern.setTransition(t);

    Triggerable triggerable = configurePatternAsTriggerable(pattern);
    drumpadRow.add(triggerable);
  }

  Triggerable configurePatternAsTriggerable(TSPattern pattern) {
    LXChannel channel = lx.engine.addChannel(new TSPattern[] { pattern });
    setupChannel(channel, false);

    pattern.onTriggerableModeEnabled();
    return pattern.getTriggerable();
  }

  void registerPatternController(String name, TSPattern pattern) {
    LXTransition t = new DissolveTransition(lx).setDuration(dissolveTime);
    pattern.setTransition(t);
    pattern.readableName = name;
    patterns.add(pattern);
  }

  /* configureEffects */

//   void registerEffect(LXEffect effect, String nfcSerialNumber) {
//     if (effect instanceof Triggerable) {
//       Triggerable triggerable = (Triggerable)effect;
//       BooleanParameter toggle = null;
//       if (drumpad != null) {
// //        toggle = drumpadTriggerablesLists[0].size() < 9 ? nfcToggles[0][drumpadTriggerablesLists[0].size()] : null;
//         drumpadTriggerablesLists[0].add(triggerable);
//       }
//     }
//   }

  void registerEffectControlParameter(LXListenableNormalizedParameter parameter) {
    registerEffectControlParameter(parameter, 0, 1, 0);
  }

  void registerEffectControlParameter(LXListenableNormalizedParameter parameter, double onValue) {
    registerEffectControlParameter(parameter, 0, onValue, 0);
  }

  void registerEffectControlParameter(LXListenableNormalizedParameter parameter, double offValue, double onValue) {
    registerEffectControlParameter(parameter, offValue, onValue, 0);
  }

  void registerEffectControlParameter(LXListenableNormalizedParameter parameter, double offValue, double onValue, int row) {
    ParameterTriggerableAdapter triggerable = new ParameterTriggerableAdapter(lx, parameter, offValue, onValue);
    BooleanParameter toggle = null;
    if (drumpad_APC != null) {
      drumpadTriggerablesLists_APC[row].add(triggerable);
    }
  }

  void registerEffectController(String name, LXEffect effect, LXListenableNormalizedParameter parameter) {
    ParameterTriggerableAdapter triggerable = new ParameterTriggerableAdapter(lx, parameter);
    TSEffectController effectController = new TSEffectController(name, effect, triggerable);

    engineController.effectControllers.add(effectController);
  }

  void configureEffects() {
    for (LXEffect effect : lx.getEffects()) {
      effect.enabled.setValue(true);
    }
  }

  /* configureBMPTool */

  void configureBMPTool() {
    bpmTool = new BPMTool(lx, effectKnobParameters);
  }

  /* configureAutomation */

  void configureAutomation() {
    // Example automation message to change master fader
    // {
    //   "message": "master/0.5",
    //   "event": "MESSAGE",
    //   "millis": 0
    // },
    lx.engine.addMessageListener(new LXEngine.MessageListener() {
      public void onMessage(LXEngine engine, String message) {
        if (message.length() > 8 && message.substring(0, 7).equals("master/")) {
          double value = Double.parseDouble(message.substring(7));
          outputBrightness.setValue(value);
        }
      }
    });

    // Automation recorders
    for (int i = 0; i < automation.length; ++i) {
      final int ii = i;
      automation[i] = new TSAutomationRecorder(lx.engine);
      lx.engine.addLoopTask(automation[i]);
      automationStop[i] = new BooleanParameter("STOP", false);
      automationStop[i].addListener(new LXParameterListener() {
        public void onParameterChanged(LXParameter parameter) {
          if (parameter.getValue() > 0) {
            automation[ii].reset();
            automation[ii].armRecord.setValue(false);
          }
        }
      });
    }

    JsonArray jsonArr = loadSavedSetFile(Config.DEFAULT_PLAYLIST);
    automation[automationSlot.getValuei()].loadJson(jsonArr);
    // slotLabel.setLabel(labels[automationSlot.getValuei()] = filename);
    automation[automationSlot.getValuei()].looping.setValue(true);
    engineController.automation = automation[automationSlot.getValuei()];

    if (Config.autoplayBMSet) {
      automation[automationSlot.getValuei()].start();
    }
  }

  /* configureTriggerables */
  
  ArrayList<Triggerable>[] drumpadTriggerablesLists_APC;
  Triggerable[][] drumpadTriggerables_APC;
  static final int drumpadRows_APC = 6;

    // The launchpad is a 9x9 pad.
    // There are some interactive elements which are "buttons" and some which are "notes" but we'll
    // deal with that in the MIDI_LP file.... consider this 9x9
  ArrayList<Triggerable>[] drumpadTriggerablesLists_LP;
  Triggerable[][] drumpadTriggerables_LP;
  static final int drumpadRows_LP = 9;
  static final int drumpadColumns_LP = 9; // this is kinda dumb

  @SuppressWarnings("unchecked")
  void configureTriggerables() {
    if (drumpad_APC != null) {
        drumpadTriggerablesLists_APC = new ArrayList[drumpadRows_APC];
        for (int i=0; i < drumpadRows_APC; i++) {
          drumpadTriggerablesLists_APC[ i ] = new ArrayList<Triggerable>();
        }
    }

    if (drumpad_LP != null) {
        drumpadTriggerablesLists_LP = new ArrayList[drumpadRows_LP];
        for (int i=0; i < drumpadRows_LP; i++) {
          drumpadTriggerablesLists_LP[ i ] = new ArrayList<Triggerable>();
        }
    }

    // These do the APC40
    registerPatternTriggerables();
    registerOneShotTriggerables();
    registerEffectTriggerables();

    // These do the Launchpad
    registerPatternTriggerables_LP();

    if (drumpad_APC != null) {
      drumpadTriggerables_APC = new Triggerable[drumpadTriggerablesLists_APC.length][];
      for (int i = 0; i < drumpadTriggerablesLists_APC.length; i++) {
        ArrayList<Triggerable> triggerablesList_APC= drumpadTriggerablesLists_APC[i];
        drumpadTriggerables_APC[i] = triggerablesList_APC.toArray(new Triggerable[triggerablesList_APC.size()]);
      }
      drumpadTriggerablesLists_APC = null;
    }

    if (drumpad_LP != null) {
      drumpadTriggerables_LP = new Triggerable[drumpadTriggerablesLists_LP.length][];
      for (int i = 0; i < drumpadTriggerablesLists_LP.length; i++) {
        ArrayList<Triggerable> triggerablesList_LP = drumpadTriggerablesLists_LP[i];
        drumpadTriggerables_LP[i] = triggerablesList_LP.toArray(new Triggerable[triggerablesList_LP.size()]);
      }
      drumpadTriggerablesLists_LP = null;
    }
  }

  /* configureMIDI */

  void configureMIDI_APC40() {

    // MIDI control
    midiEngine_APC = new MidiEngine_APC(lx, effectKnobParameters, drumpad_APC, drumpadVelocity, 
      previewChannels, bpmTool, uiDeck, outputBrightness, 
      automationSlot, automation, automationStop);
  }

  void configureMIDI_Launchpad() {

    // MIDI control
    midiEngine_LP = new MidiEngine_LP(lx, this, effectKnobParameters, drumpadVelocity, 
      previewChannels, bpmTool, uiDeck, outputBrightness, 
      automationSlot, automation, automationStop);

  }

  /* configureExternalOutput */

  void configureExternalOutput() {
    // Output stage
    try {
      output = new LXDatagramOutput(lx);
      datagrams = new LXDatagram[model.clusters.size()];
      int ci = 0;
      for (Cluster cluster : model.clusters) {
        output.addDatagram(datagrams[ci++] = Output.clusterDatagram(cluster).setAddress(cluster.getIpAddress()));
      }
      outputBrightness.parameters.add(output.brightness);
      output.enabled.setValue(Config.enableLiveOutput);
      lx.addOutput(output);
      output.start();
    } catch (Exception x) {
      System.out.println(x);
    }
  }

  /* configureFadeCandyOutput -- no output defined for fade candy yet */

  /* configureServer */

  void configureServer() {
    new AppServer(lx, engineController).start();
  }
}

class EngineController {
  LX lx;

  int baseChannelIndex;
  int numChannels;

  int startEffectIndex;
  int endEffectIndex;

  boolean isAutoplaying;
  TSAutomationRecorder automation;
  boolean[] previousChannelIsOn;

  ArrayList<TSEffectController> effectControllers = new ArrayList<TSEffectController>();
  int activeEffectControllerIndex = -1;

  SpeedEffect speedEffect;
  SpinEffect spinEffect;
  BlurEffect blurEffect;
  ScrambleEffect scrambleEffect;
  BrightnessScaleEffect masterBrightnessEffect;

  EngineController(LX lx) {
    this.lx = lx;
  }

  List<LXChannel> getChannels() {
    return lx.engine.getChannels().subList(baseChannelIndex, baseChannelIndex + numChannels);
  }

  void setChannelPattern(int channelIndex, int patternIndex) {
    if (patternIndex == -1) {
      patternIndex = 0;
    } else {
      patternIndex++;
    }
    lx.engine.getChannel(channelIndex).goIndex(patternIndex);
  }

  void setChannelVisibility(int channelIndex, double visibility) {
    lx.engine.getChannel(channelIndex).getFader().setValue(visibility);
  }

  void setActiveColorEffect(int effectIndex) {
    if (activeEffectControllerIndex == effectIndex) {
      return;
    }
    if (activeEffectControllerIndex != -1) {
      TSEffectController effectController = effectControllers.get(activeEffectControllerIndex);
      effectController.setEnabled(false);
    }
    activeEffectControllerIndex = effectIndex;
    if (activeEffectControllerIndex != -1) {
      TSEffectController effectController = effectControllers.get(activeEffectControllerIndex);
      effectController.setEnabled(true);
    }
  }

  void setSpeed(double amount) {
    speedEffect.speed.setValue(amount);
  }

  void setSpin(double amount) {
    spinEffect.spin.setValue(amount);
  }

  void setBlur(double amount) {
    blurEffect.amount.setValue(amount);
  }

  void setScramble(double amount) {
    scrambleEffect.amount.setValue(amount);
  }

  void setMasterBrightness(double amount) {
    masterBrightnessEffect.amount.setValue(amount);
  }

  void setAutoplay(boolean autoplay) {
    setAutoplay(autoplay, false);
  }

  void setAutoplay(boolean autoplay, boolean forceUpdate) {
    if (autoplay != isAutoplaying || forceUpdate) {
      isAutoplaying = autoplay;
      automation.setPaused(!autoplay);

      if (previousChannelIsOn == null) {
        previousChannelIsOn = new boolean[lx.engine.getChannels().size()];
        for (LXChannel channel : lx.engine.getChannels()) {
          previousChannelIsOn[channel.getIndex()] = channel.enabled.isOn();
        }
      }

      for (LXChannel channel : lx.engine.getChannels()) {
        boolean toEnable;
        if (channel.getIndex() < baseChannelIndex) {
          toEnable = autoplay;
        } else if (channel.getIndex() < baseChannelIndex + numChannels) {
          toEnable = !autoplay;
        } else {
          toEnable = autoplay;
        }

        if (toEnable) {
          channel.enabled.setValue(previousChannelIsOn[channel.getIndex()]);
        } else {
          previousChannelIsOn[channel.getIndex()] = channel.enabled.isOn();
          channel.enabled.setValue(false);
        }
      }

      for (int i = 0; i < lx.engine.getEffects().size(); i++) {
        LXEffect effect = lx.engine.getEffects().get(i);
        if (i < startEffectIndex) {
          effect.enabled.setValue(autoplay);
        } else if (i < endEffectIndex) {
          effect.enabled.setValue(!autoplay);
        }
      }
    }
  }
}

class CrownTransition extends LXTransition {
  
  private final LXChannel channel;
  
  public final DiscreteParameter blendMode = new DiscreteParameter("MODE", 4);
  private LXColor.Blend blendType = LXColor.Blend.ADD;

  final BasicParameter fade = new BasicParameter("FADE", 1);
    
  CrownTransition(LX lx, LXChannel channel) {
    super(lx);
    addParameter(blendMode);
    
    this.channel = channel;
    blendMode.addListener(new LXParameterListener() {
      public void onParameterChanged(LXParameter parameter) {
        switch (blendMode.getValuei()) {
        case 0: blendType = LXColor.Blend.ADD; break;
        case 1: blendType = LXColor.Blend.MULTIPLY; break;
        case 2: blendType = LXColor.Blend.LIGHTEST; break;
        case 3: blendType = LXColor.Blend.SUBTRACT; break;
        }
      }
    });
  }
  
  protected void computeBlend(int[] c1, int[] c2, double progress) {
    if (progress == 0) {
      for (int i = 0; i < colors.length; ++i) {
        colors[i] = c1[i];
      }
    } else if (progress == 1) {
      for (int i = 0; i < colors.length; ++i) {
        int color2 = (blendType == LXColor.Blend.SUBTRACT) ? LX.hsb(0, 0, LXColor.b(c2[i])) : c2[i]; 
        colors[i] = LXColor.blend(c1[i], color2, this.blendType);
      }
    } else {
      for (int i = 0; i < colors.length; ++i) {
        int color2 = (blendType == LXColor.Blend.SUBTRACT) ? LX.hsb(0, 0, LXColor.b(c2[i])) : c2[i];
        colors[i] = LXColor.lerp(c1[i], LXColor.blend(c1[i], color2, this.blendType), progress);
      }
    }
  }
}

//
// We find some midi events are comming in on non-LX threads and thus
// causing threading hazards. We need to transition over to 
// LX using a LXLoopTask, which will read from the queue

class MIDIEventTask implements LXLoopTask {

  static class MIDIEvent {

    // Should use an Enum but I'm in a hurry and don't remember how
    // 1 == onPressed 2 == onReleased
    public int type;
    public int x;
    public int y;
    public float velocity;
    public long timestamp;

    MIDIEvent(int type, int x, int y, float velocity, long timestamp) {
      this.type = type;
      this.x = x;
      this.y = y;
      this.velocity = velocity;
      this.timestamp = timestamp;
    }

  };

  ArrayDeque< MIDIEvent > q;

  Drumpad drumpad;

  MIDIEventTask(Drumpad drumpad) {
    this.drumpad = drumpad;
    this.q = new ArrayDeque<MIDIEvent>();
  }

  public void add( MIDIEvent e ) {
    q.addLast(e);
  }

  public void loop(double deltaMs) {
    MIDIEvent e = q.pollFirst();
    if (e == null) return;

    if (e.type == 1) {
      drumpad.padTriggered(e.x, e.y, e.velocity );
    }
    else if (e.type == 2) { // released 
      drumpad.padReleased(e.x, e.y);
    }
    else {
      System.out.println( " receved event with unknown type   "+e.type);
    }
  }
}


class TSAutomationRecorder extends LXAutomationRecorder {

  boolean isPaused;

  TSAutomationRecorder(LXEngine engine) {
    super(engine);
  }

  @Override
  protected void onStart() {
    super.onStart();
    isPaused = false;
  }

  public void setPaused(boolean paused) {
    if (!paused && !isRunning()) {
      start();
    }
    isPaused = paused;
  }

  @Override
  public void loop(double deltaMs) {
    if (!isPaused) {
      super.loop(deltaMs);
    }
  }
}

class BooleanParameterProxy extends BooleanParameter {

  final List<BooleanParameter> parameters = new ArrayList<BooleanParameter>();

  BooleanParameterProxy() {
    super("Proxy", true);
  }

  protected double updateValue(double value) {
    for (BooleanParameter parameter : parameters) {
      parameter.setValue(value);
    }
    return value;
  }
}

class BasicParameterProxy extends BasicParameter {

  final List<BasicParameter> parameters = new ArrayList<BasicParameter>();

  BasicParameterProxy(double value) {
    super("Proxy", value);
  }

  protected double updateValue(double value) {
    for (BasicParameter parameter : parameters) {
      parameter.setValue(value);
    }
    return value;
  }
}
