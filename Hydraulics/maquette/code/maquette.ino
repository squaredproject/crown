// basic arduino code
// XXX add some arduino headers here...

#define NUM_TOWERS 4
#define NUM_JOINTS 3
#define MIN_POT 384
#define MAX_POT 640

uint16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}}};
// XXX - above. At some point I use real values. Not just yet though.

int modelPosition[NUM_TOWERS][NUM_JOINTS];
int pinMapping[NUM_TOWERS][NUM_JOINTS] = { {A0, A1, A2},
                                          {A3, A4, A5},
                                          {A6, A7, A8},
                                          {A9, A10, A11} };
                                          
                                          

static int16_t maquetteToModel(uint16_t pos, int i, int j);
static uint16_t clamp(uint16_t value, uint16_t max, uint16_t min);
static void targetSculpture();
static void broadcastModelPosition();

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
    delay(100); // every 10th of a second...
    // Typical wait appears to be about 10X per second... which is fine with me.
    for (int i=0; i<NUM_TOWERS; i++) {
        for (int j=0; j<NUM_JOINTS; j++) {
            uint16_t potValue  = analogRead(pinMapping[i][j]);
            modelPosition[i][j] = maquetteToModel(potValue, i, j);
        }
    }
    broadcastModelPosition();
    targetSculpture();
}

/* Translate between potentiometer reading (-16K : +16K) 
   to model position (-200 to +200). Note that not all
   values read by the potentiometer are valid - we clamp the 
   value between MAX_POT and MIN_POT */

static int16_t maquetteToModel(uint16_t pos, int i, int j) {
    int16_t value = clamp(pos, MIN_POT, MAX_POT) - 512;
    
    return  ((long)value * (long)(jointRange[i][j][1] - jointRange[i][j][0]))/(long)(MAX_POT - MIN_POT);
}


static uint16_t clamp(uint16_t value, uint16_t min, uint16_t max) {
    return value > max ? max : (value < min ? min : value);
}

// Send target position to sculpture
static void targetSculpture() {
    char modelString[256];
    char *ptr = modelString;
    for (int i=0; i<NUM_TOWERS; i++){
        for (int j=0; j<NUM_JOINTS; j++) {
            sprintf(ptr, "<%d%dt%d>", i, j, modelPosition[i][j]);
            ptr += strlen(ptr);
        }
    }
    sprintf(ptr, "\r\n");
    Serial.print(modelString);
}

// Let the rest of the world know what we're doing
// Here I'm going to work in JSON, since that's easiest for me to parse
static void broadcastModelPosition() {
    char modelString[256];
    char *ptr = modelString;
    sprintf(ptr, "[");
    ptr += strlen(ptr);
    for (int i=0; i<NUM_TOWERS; i++){
        sprintf(ptr, "{'tower': %d, 'joints':[", i);
        ptr += strlen(ptr);
        for (int j=0; j<NUM_JOINTS; j++) {
            sprintf(ptr, "%d,", modelPosition[i][j]);
            ptr += strlen(ptr);
        }
        ptr -= 1; // cut trailing ','
        sprintf(ptr, "]},");
        ptr += strlen(ptr);
    }
    ptr -= 1; // cut trailing '\n'
    sprintf(ptr, "]\n");
    Serial1.print(modelString); 
}
