#!/usr/bin/python
# makesine.py
# Python program to generate sineusoidal lookup table
# generates an output file "sine.h"


import math

# length of table
table_length = 1024

# number of bits in amplitude (sign will add one bit to the output size)
table_amplitude_bits = 10;

# maximum value in table
table_amplitude = 2 ** table_amplitude_bits;

# output file name
outfilename = "sinetable.h"


#write table to this file
outfile = open(outfilename, "w")

# print header
outfile.write("/* sine.h autogenerated by makesine.py */\n")
outfile.write("/* table length is %d */\n" % table_length)
outfile.write(""" /*
To use: 'python makesine.py'
Then insert in your code

#include <avr/pgmspace.h>
#include "sinetable.h"

uint16_t sineval;

(code)

sineval = (uint16_t)pgm_read_byte(&stab[phase]);

*/
""")

outfile.write("#define TABLE_LENGTH (%d)\n" % table_length)
outfile.write("#define TABLE_AMPLITUDE_BITS  (%d)\n" % int(table_amplitude_bits))
outfile.write("#define TABLE_AMPLITUDE  (%d)\n" % int(table_amplitude))
outfile.write("\n")
outfile.write("const int16_t stab[TABLE_LENGTH] PROGMEM = { \n")

for i in range(table_length):
    phase = 2*math.pi*i/float(table_length)
    outfile.write("%4d" %
                  int(math.floor(table_amplitude*math.sin(phase))))
    if i < table_length -1:
        outfile.write(", /* %4d */\n" % i)
    else:
        outfile.write("}; /* %4d */\n" % i)


print 'created sine table file "%s" with %d entries' % (outfilename,table_length)
outfile.close()