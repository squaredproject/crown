/* sine.h autogenerated by makesine.py */
/* table length is 360 */
/*
To use: 'python makesine.py'
Then insert in your code

#include "sine.h"
#include <avr/pgmspace.h>

uint16_t sineval;

(code)

sineval = (uint16_t)pgm_read_byte(&stab[phase]);

*/
#include <avr/pgmspace.h>

#define TABLENGTH (360)
#define TABLEMAX (1024)
const uint16_t stab[TABLENGTH] PROGMEM = {512,  /*    0 */
                                          520,  /*    1 */
                                          529,  /*    2 */
                                          538,  /*    3 */
                                          547,  /*    4 */
                                          556,  /*    5 */
                                          565,  /*    6 */
                                          574,  /*    7 */
                                          583,  /*    8 */
                                          592,  /*    9 */
                                          600,  /*   10 */
                                          609,  /*   11 */
                                          618,  /*   12 */
                                          627,  /*   13 */
                                          635,  /*   14 */
                                          644,  /*   15 */
                                          653,  /*   16 */
                                          661,  /*   17 */
                                          670,  /*   18 */
                                          678,  /*   19 */
                                          687,  /*   20 */
                                          695,  /*   21 */
                                          703,  /*   22 */
                                          712,  /*   23 */
                                          720,  /*   24 */
                                          728,  /*   25 */
                                          736,  /*   26 */
                                          744,  /*   27 */
                                          752,  /*   28 */
                                          760,  /*   29 */
                                          768,  /*   30 */
                                          775,  /*   31 */
                                          783,  /*   32 */
                                          790,  /*   33 */
                                          798,  /*   34 */
                                          805,  /*   35 */
                                          812,  /*   36 */
                                          820,  /*   37 */
                                          827,  /*   38 */
                                          834,  /*   39 */
                                          841,  /*   40 */
                                          847,  /*   41 */
                                          854,  /*   42 */
                                          861,  /*   43 */
                                          867,  /*   44 */
                                          874,  /*   45 */
                                          880,  /*   46 */
                                          886,  /*   47 */
                                          892,  /*   48 */
                                          898,  /*   49 */
                                          904,  /*   50 */
                                          909,  /*   51 */
                                          915,  /*   52 */
                                          920,  /*   53 */
                                          926,  /*   54 */
                                          931,  /*   55 */
                                          936,  /*   56 */
                                          941,  /*   57 */
                                          946,  /*   58 */
                                          950,  /*   59 */
                                          955,  /*   60 */
                                          959,  /*   61 */
                                          964,  /*   62 */
                                          968,  /*   63 */
                                          972,  /*   64 */
                                          976,  /*   65 */
                                          979,  /*   66 */
                                          983,  /*   67 */
                                          986,  /*   68 */
                                          989,  /*   69 */
                                          993,  /*   70 */
                                          996,  /*   71 */
                                          998,  /*   72 */
                                          1001, /*   73 */
                                          1004, /*   74 */
                                          1006, /*   75 */
                                          1008, /*   76 */
                                          1010, /*   77 */
                                          1012, /*   78 */
                                          1014, /*   79 */
                                          1016, /*   80 */
                                          1017, /*   81 */
                                          1019, /*   82 */
                                          1020, /*   83 */
                                          1021, /*   84 */
                                          1022, /*   85 */
                                          1022, /*   86 */
                                          1023, /*   87 */
                                          1023, /*   88 */
                                          1023, /*   89 */
                                          1024, /*   90 */
                                          1023, /*   91 */
                                          1023, /*   92 */
                                          1023, /*   93 */
                                          1022, /*   94 */
                                          1022, /*   95 */
                                          1021, /*   96 */
                                          1020, /*   97 */
                                          1019, /*   98 */
                                          1017, /*   99 */
                                          1016, /*  100 */
                                          1014, /*  101 */
                                          1012, /*  102 */
                                          1010, /*  103 */
                                          1008, /*  104 */
                                          1006, /*  105 */
                                          1004, /*  106 */
                                          1001, /*  107 */
                                          998,  /*  108 */
                                          996,  /*  109 */
                                          993,  /*  110 */
                                          989,  /*  111 */
                                          986,  /*  112 */
                                          983,  /*  113 */
                                          979,  /*  114 */
                                          976,  /*  115 */
                                          972,  /*  116 */
                                          968,  /*  117 */
                                          964,  /*  118 */
                                          959,  /*  119 */
                                          955,  /*  120 */
                                          950,  /*  121 */
                                          946,  /*  122 */
                                          941,  /*  123 */
                                          936,  /*  124 */
                                          931,  /*  125 */
                                          926,  /*  126 */
                                          920,  /*  127 */
                                          915,  /*  128 */
                                          909,  /*  129 */
                                          904,  /*  130 */
                                          898,  /*  131 */
                                          892,  /*  132 */
                                          886,  /*  133 */
                                          880,  /*  134 */
                                          874,  /*  135 */
                                          867,  /*  136 */
                                          861,  /*  137 */
                                          854,  /*  138 */
                                          847,  /*  139 */
                                          841,  /*  140 */
                                          834,  /*  141 */
                                          827,  /*  142 */
                                          820,  /*  143 */
                                          812,  /*  144 */
                                          805,  /*  145 */
                                          798,  /*  146 */
                                          790,  /*  147 */
                                          783,  /*  148 */
                                          775,  /*  149 */
                                          768,  /*  150 */
                                          760,  /*  151 */
                                          752,  /*  152 */
                                          744,  /*  153 */
                                          736,  /*  154 */
                                          728,  /*  155 */
                                          720,  /*  156 */
                                          712,  /*  157 */
                                          703,  /*  158 */
                                          695,  /*  159 */
                                          687,  /*  160 */
                                          678,  /*  161 */
                                          670,  /*  162 */
                                          661,  /*  163 */
                                          653,  /*  164 */
                                          644,  /*  165 */
                                          635,  /*  166 */
                                          627,  /*  167 */
                                          618,  /*  168 */
                                          609,  /*  169 */
                                          600,  /*  170 */
                                          592,  /*  171 */
                                          583,  /*  172 */
                                          574,  /*  173 */
                                          565,  /*  174 */
                                          556,  /*  175 */
                                          547,  /*  176 */
                                          538,  /*  177 */
                                          529,  /*  178 */
                                          520,  /*  179 */
                                          512,  /*  180 */
                                          503,  /*  181 */
                                          494,  /*  182 */
                                          485,  /*  183 */
                                          476,  /*  184 */
                                          467,  /*  185 */
                                          458,  /*  186 */
                                          449,  /*  187 */
                                          440,  /*  188 */
                                          431,  /*  189 */
                                          423,  /*  190 */
                                          414,  /*  191 */
                                          405,  /*  192 */
                                          396,  /*  193 */
                                          388,  /*  194 */
                                          379,  /*  195 */
                                          370,  /*  196 */
                                          362,  /*  197 */
                                          353,  /*  198 */
                                          345,  /*  199 */
                                          336,  /*  200 */
                                          328,  /*  201 */
                                          320,  /*  202 */
                                          311,  /*  203 */
                                          303,  /*  204 */
                                          295,  /*  205 */
                                          287,  /*  206 */
                                          279,  /*  207 */
                                          271,  /*  208 */
                                          263,  /*  209 */
                                          255,  /*  210 */
                                          248,  /*  211 */
                                          240,  /*  212 */
                                          233,  /*  213 */
                                          225,  /*  214 */
                                          218,  /*  215 */
                                          211,  /*  216 */
                                          203,  /*  217 */
                                          196,  /*  218 */
                                          189,  /*  219 */
                                          182,  /*  220 */
                                          176,  /*  221 */
                                          169,  /*  222 */
                                          162,  /*  223 */
                                          156,  /*  224 */
                                          149,  /*  225 */
                                          143,  /*  226 */
                                          137,  /*  227 */
                                          131,  /*  228 */
                                          125,  /*  229 */
                                          119,  /*  230 */
                                          114,  /*  231 */
                                          108,  /*  232 */
                                          103,  /*  233 */
                                          97,   /*  234 */
                                          92,   /*  235 */
                                          87,   /*  236 */
                                          82,   /*  237 */
                                          77,   /*  238 */
                                          73,   /*  239 */
                                          68,   /*  240 */
                                          64,   /*  241 */
                                          59,   /*  242 */
                                          55,   /*  243 */
                                          51,   /*  244 */
                                          47,   /*  245 */
                                          44,   /*  246 */
                                          40,   /*  247 */
                                          37,   /*  248 */
                                          34,   /*  249 */
                                          30,   /*  250 */
                                          27,   /*  251 */
                                          25,   /*  252 */
                                          22,   /*  253 */
                                          19,   /*  254 */
                                          17,   /*  255 */
                                          15,   /*  256 */
                                          13,   /*  257 */
                                          11,   /*  258 */
                                          9,    /*  259 */
                                          7,    /*  260 */
                                          6,    /*  261 */
                                          4,    /*  262 */
                                          3,    /*  263 */
                                          2,    /*  264 */
                                          1,    /*  265 */
                                          1,    /*  266 */
                                          0,    /*  267 */
                                          0,    /*  268 */
                                          0,    /*  269 */
                                          0,    /*  270 */
                                          0,    /*  271 */
                                          0,    /*  272 */
                                          0,    /*  273 */
                                          1,    /*  274 */
                                          1,    /*  275 */
                                          2,    /*  276 */
                                          3,    /*  277 */
                                          4,    /*  278 */
                                          6,    /*  279 */
                                          7,    /*  280 */
                                          9,    /*  281 */
                                          11,   /*  282 */
                                          13,   /*  283 */
                                          15,   /*  284 */
                                          17,   /*  285 */
                                          19,   /*  286 */
                                          22,   /*  287 */
                                          25,   /*  288 */
                                          27,   /*  289 */
                                          30,   /*  290 */
                                          34,   /*  291 */
                                          37,   /*  292 */
                                          40,   /*  293 */
                                          44,   /*  294 */
                                          47,   /*  295 */
                                          51,   /*  296 */
                                          55,   /*  297 */
                                          59,   /*  298 */
                                          64,   /*  299 */
                                          68,   /*  300 */
                                          73,   /*  301 */
                                          77,   /*  302 */
                                          82,   /*  303 */
                                          87,   /*  304 */
                                          92,   /*  305 */
                                          97,   /*  306 */
                                          103,  /*  307 */
                                          108,  /*  308 */
                                          114,  /*  309 */
                                          119,  /*  310 */
                                          125,  /*  311 */
                                          131,  /*  312 */
                                          137,  /*  313 */
                                          143,  /*  314 */
                                          149,  /*  315 */
                                          156,  /*  316 */
                                          162,  /*  317 */
                                          169,  /*  318 */
                                          176,  /*  319 */
                                          182,  /*  320 */
                                          189,  /*  321 */
                                          196,  /*  322 */
                                          203,  /*  323 */
                                          211,  /*  324 */
                                          218,  /*  325 */
                                          225,  /*  326 */
                                          233,  /*  327 */
                                          240,  /*  328 */
                                          248,  /*  329 */
                                          255,  /*  330 */
                                          263,  /*  331 */
                                          271,  /*  332 */
                                          279,  /*  333 */
                                          287,  /*  334 */
                                          295,  /*  335 */
                                          303,  /*  336 */
                                          311,  /*  337 */
                                          320,  /*  338 */
                                          328,  /*  339 */
                                          336,  /*  340 */
                                          345,  /*  341 */
                                          353,  /*  342 */
                                          362,  /*  343 */
                                          370,  /*  344 */
                                          379,  /*  345 */
                                          388,  /*  346 */
                                          396,  /*  347 */
                                          405,  /*  348 */
                                          414,  /*  349 */
                                          423,  /*  350 */
                                          431,  /*  351 */
                                          440,  /*  352 */
                                          449,  /*  353 */
                                          458,  /*  354 */
                                          467,  /*  355 */
                                          476,  /*  356 */
                                          485,  /*  357 */
                                          494,  /*  358 */
                                          503}; /*  359 */
