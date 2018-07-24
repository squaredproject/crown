#!/usr/bin/python
# makesine.py
# Python program to generate sineusoidal lookup table
# generates an output file "sine.h"


import math

# length of table
tablength = 360
# maximum value in table
tabmax = 1023
# output file name
outfilename = "sine.h"


#write table to this file
outfile = open(outfilename, "w")

# print header
outfile.write("/* sine.h autogenerated by makesine.py */\n")
outfile.write("/* table length is %d */\n" % tablength)
outfile.write(""" /*
To use: 'python makesine.py'
Then insert in your code

#include <avr/pgmspace.h>
#include "sine.h"

uint16_t sineval;

(code)

sineval = (uint16_t)pgm_read_byte(&stab[phase]);

*/
""")

halfmax = math.floor(tabmax/2)
outfile.write("#define TABLENGTH (%d)\n" % tablength)
outfile.write("#define TABLEMAX  (%d)\n" % int(tabmax))
outfile.write("const uint16_t stab[TABLENGTH] PROGMEM = { \n")

for i in range(tablength):
    phase = 2*math.pi*i/float(tablength)
    outfile.write("%4d" %
                  int(math.floor(halfmax*math.sin(phase) + halfmax)))
    if i < tablength -1:
        outfile.write(", /* %4d */\n" % i)
    else:
        outfile.write("}; /* %4d */\n" % i)


print 'created sine table file "%s" with %d entries' % (outfilename,tablength)
outfile.close()