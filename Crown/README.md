# README

Sets for live play, the details of the color map, lessons learned

## Requirements
Java 8, 9, or 10
Gradle
Processing 3.0
git

## How to Run
You can either run through the Processing  or run the app headlessly.  Use the Processing application when you want to use the UI and DJ patterns.  headless mode is only for when the crown app is running continuously on an odroid without a screen (showtime)

### HOWTO Run Interactively with Processing
1.  Download Crown repo
    - git clone https://github.com/squaredproject/crown.git
2.  Open Processing 3.0
    - click File->Open
    - navigate to Crown directory
    - click on Crown.pde 
    - click on the arrow button to Run
3.  Once Processing has loaded the app, it should show you the Crown software and you can start making patterns

### HOWTO Run Headlessly
1.  Download Crown repo from github
    - git clone https://github.com/squaredproject/crown.git
2.  Open terminal and navigate to repo directory
    - eg. cd ~/crown/Crown
3.  Compile java application using gradle
    - gradle build
4.  Excute shell script to run headlessly
    - ./run.sh
    - confirm script out says "LX Engine Started" at the end
5.  Optional: configure machine to run crown on startup using crown.servoce


### How to change NDP IP addresses
The IP address for every NDB is saved in a config file called data/crown_clusters.json
If you change the NDBs conncted to the sculpture, you will need to:
1.  Edit data/crown_clusters.json and add your specific NDP IP address in the right location
2.  Recompile code using ./compile.sh OR restart Processing
2a. Optional - restart NDB by temporarily unplugging it from the power
3.  Confirm new NDB LEDS are receiving data and LEDs are turning on as expected

## SAVING SETS FOR LIVE PLAY

### READ THIS

If you don't follow these instructions, Processing will crash.

The key sequence for recording a "set" using the automation system is as follows:

In the lower right corner:
- stop any previous automation using stop
- get ready to start by pressing ARM
- press PLAY then start your set
- press STOP to stop your set
- press SAVE to save it

There are other key sequences that crash the system, and we don't have time to figure them
out now ( well, we tried, and couldn't figure it out :-) ). 

Such as: ARM - SAVE, that crashes.

## LED Map

The IP addresses of the NDBs are set in data/crown_clusters.json. There are only 5, and they each have their
own IP address.

### Fence

The fence is controlled by two NDBs. By convention, the NDB will be placed in the south-west corner.

The NDBs will have 6 lines active. 

The first NDB will do all counter-clockwise LEDs. That is noted as "index 0" in the config file.

The second NDB will do clockwise LEDs. that will be noted as "index 1" in the config file.


There will be six rows of LEDs.

Each edge of the fence will have 30 LEDs per row - but the row will continue around a single corner, and connect 30 more LEDs.

Thus, the NDB will be configured with 60 LEDs per channel.

In the map, the order is:

- Top Counterclockwise
- Top Clockwise
- Next one down, counterclockwise
- Next one down, clockwise

Note: the reason we have two NDBs is entirely related to the issue of maximum packet size.
If you have too many NDBs, you will find 

### Fence NDB map

Each control channel should be programmed with 1-60 lights, in ascending order.

There is one complexity, which is one LED in the top line at each corner ( 8 total, 2 per side ) may need to be black.

Right now, I'm assuming we will have the light, but simply tuck it away. We may need to remove that light from the model.

### Tower

Each tower has 3 segments, each segment has 12 LEDs.

There are also 12 channels of each NDB used. By convention, these will go from the eastern-most string,
rotating around.

Since there are 36 LEDs per channel, each tower NDB should be configured with 36 lights, sequentially from channel 1 to channel 12.

The LEDs were supposed to be 8 inches apart, they are actually about 4 inches center-to-center

# LESSONS LEARNED

## Use LXStudio ( soon )

There is a lot of code and improvements in LX STudio, which is the project Mark started doing when he
had to build this UI over and over. It really does look like you move the Model file and some of the UI
elements and the Patterns files, and "bob is your uncle". It will be worth a try Real Soon Now.

## Midi

Using midi controllers is really cool! There are some annoyances.

### APC40

Squared uses this controller called an APC40 built by Akai. The annoyance with the APC40 is that it's
built for Ableton, and in order to get it to work at all, you have to send a particular SysEx which
puts it in Ableton mode. This is ONLY an annoyance created by the manufacturer, to cause people
to NOT take this rather amazing hardware and use it.

Thus, there has to be code in MIDI.java to send that one little SysEx.

The annoying wrinkle is that MacOS's java is messed up about sending SysEx packets.... even in 2018.
There is a library called 'mmj' which is commonly used to send these packets.... after which you can
use standard midi because it's in the right mode.

The annoyance is even just _having this jar in the path_ prevents midi from working on other platforms,
because the midi iterator hits the mmj library which is macos only, and creates a linker error.

Thus, this jar has to be in the path ONLY on macos and needs to be moved out otherwise.

Todo: there is a replacement for this Jar called CoreMidi4J which is maintained and works with "modern java".
Switching over to that is a good idea, eventually.

### LaunchPad Mini

The Launchpad is a cool little device also. Very dumb, no programming, just lots of buttons.

We are using a library LP4J which seems to work absolutely a treat. On Linux, it just fires up.
You can validate that the kernel portions are working by first doing an 'lsusb' to validate that
the device is showing up in USb, then doing ALSA commands to see it in hte list of Alsa midi,
then it should show up in the java iterators.


