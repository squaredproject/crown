# LED Map

The IP addresses of the NDBs are set in data/crown_clusters.json. There are only 5, and they each have their
own IP address.

## Fence

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

## Tower

Each tower has 3 segments, each segment has 12 LEDs.

There are also 12 channels of each NDB used. By convention, these will go from the eastern-most string,
rotating around.

Since there are 36 LEDs per channel, each tower NDB should be configured with 36 lights, sequentially from channel 1 to channel 12.