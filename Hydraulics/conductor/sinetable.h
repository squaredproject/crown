/* sine.h autogenerated by makesine.py */
/* table length is 1024 */
 /*
To use: 'python makesine.py'
Then insert in your code

#include <avr/pgmspace.h>
#include "sinetable.h"

uint16_t sineval;

(code)

sineval = (uint16_t)pgm_read_byte(&stab[phase]);

*/

#ifndef SINETABLE_H_
#define SINETABLE_H_

// dual defined here and in conductor.h . Change both if needed.
#ifndef TABLE_LENGTH
#define TABLE_LENGTH 1024
#endif /* TABLE_LENGTH */

#define TABLE_AMPLITUDE_BITS  (10)
#define TABLE_AMPLITUDE  (1024)

const int16_t stab[TABLE_LENGTH] PROGMEM = { 
   0, /*    0 */
   6, /*    1 */
  12, /*    2 */
  18, /*    3 */
  25, /*    4 */
  31, /*    5 */
  37, /*    6 */
  43, /*    7 */
  50, /*    8 */
  56, /*    9 */
  62, /*   10 */
  69, /*   11 */
  75, /*   12 */
  81, /*   13 */
  87, /*   14 */
  94, /*   15 */
 100, /*   16 */
 106, /*   17 */
 112, /*   18 */
 119, /*   19 */
 125, /*   20 */
 131, /*   21 */
 137, /*   22 */
 144, /*   23 */
 150, /*   24 */
 156, /*   25 */
 162, /*   26 */
 168, /*   27 */
 175, /*   28 */
 181, /*   29 */
 187, /*   30 */
 193, /*   31 */
 199, /*   32 */
 205, /*   33 */
 212, /*   34 */
 218, /*   35 */
 224, /*   36 */
 230, /*   37 */
 236, /*   38 */
 242, /*   39 */
 248, /*   40 */
 254, /*   41 */
 260, /*   42 */
 267, /*   43 */
 273, /*   44 */
 279, /*   45 */
 285, /*   46 */
 291, /*   47 */
 297, /*   48 */
 303, /*   49 */
 309, /*   50 */
 315, /*   51 */
 321, /*   52 */
 327, /*   53 */
 333, /*   54 */
 339, /*   55 */
 344, /*   56 */
 350, /*   57 */
 356, /*   58 */
 362, /*   59 */
 368, /*   60 */
 374, /*   61 */
 380, /*   62 */
 386, /*   63 */
 391, /*   64 */
 397, /*   65 */
 403, /*   66 */
 409, /*   67 */
 414, /*   68 */
 420, /*   69 */
 426, /*   70 */
 432, /*   71 */
 437, /*   72 */
 443, /*   73 */
 449, /*   74 */
 454, /*   75 */
 460, /*   76 */
 466, /*   77 */
 471, /*   78 */
 477, /*   79 */
 482, /*   80 */
 488, /*   81 */
 493, /*   82 */
 499, /*   83 */
 504, /*   84 */
 510, /*   85 */
 515, /*   86 */
 521, /*   87 */
 526, /*   88 */
 531, /*   89 */
 537, /*   90 */
 542, /*   91 */
 547, /*   92 */
 553, /*   93 */
 558, /*   94 */
 563, /*   95 */
 568, /*   96 */
 574, /*   97 */
 579, /*   98 */
 584, /*   99 */
 589, /*  100 */
 594, /*  101 */
 599, /*  102 */
 604, /*  103 */
 609, /*  104 */
 615, /*  105 */
 620, /*  106 */
 625, /*  107 */
 629, /*  108 */
 634, /*  109 */
 639, /*  110 */
 644, /*  111 */
 649, /*  112 */
 654, /*  113 */
 659, /*  114 */
 664, /*  115 */
 668, /*  116 */
 673, /*  117 */
 678, /*  118 */
 683, /*  119 */
 687, /*  120 */
 692, /*  121 */
 696, /*  122 */
 701, /*  123 */
 706, /*  124 */
 710, /*  125 */
 715, /*  126 */
 719, /*  127 */
 724, /*  128 */
 728, /*  129 */
 732, /*  130 */
 737, /*  131 */
 741, /*  132 */
 745, /*  133 */
 750, /*  134 */
 754, /*  135 */
 758, /*  136 */
 762, /*  137 */
 767, /*  138 */
 771, /*  139 */
 775, /*  140 */
 779, /*  141 */
 783, /*  142 */
 787, /*  143 */
 791, /*  144 */
 795, /*  145 */
 799, /*  146 */
 803, /*  147 */
 807, /*  148 */
 811, /*  149 */
 814, /*  150 */
 818, /*  151 */
 822, /*  152 */
 826, /*  153 */
 829, /*  154 */
 833, /*  155 */
 837, /*  156 */
 840, /*  157 */
 844, /*  158 */
 847, /*  159 */
 851, /*  160 */
 854, /*  161 */
 858, /*  162 */
 861, /*  163 */
 865, /*  164 */
 868, /*  165 */
 871, /*  166 */
 875, /*  167 */
 878, /*  168 */
 881, /*  169 */
 884, /*  170 */
 887, /*  171 */
 890, /*  172 */
 894, /*  173 */
 897, /*  174 */
 900, /*  175 */
 903, /*  176 */
 906, /*  177 */
 908, /*  178 */
 911, /*  179 */
 914, /*  180 */
 917, /*  181 */
 920, /*  182 */
 922, /*  183 */
 925, /*  184 */
 928, /*  185 */
 930, /*  186 */
 933, /*  187 */
 936, /*  188 */
 938, /*  189 */
 941, /*  190 */
 943, /*  191 */
 946, /*  192 */
 948, /*  193 */
 950, /*  194 */
 953, /*  195 */
 955, /*  196 */
 957, /*  197 */
 959, /*  198 */
 962, /*  199 */
 964, /*  200 */
 966, /*  201 */
 968, /*  202 */
 970, /*  203 */
 972, /*  204 */
 974, /*  205 */
 976, /*  206 */
 978, /*  207 */
 979, /*  208 */
 981, /*  209 */
 983, /*  210 */
 985, /*  211 */
 986, /*  212 */
 988, /*  213 */
 990, /*  214 */
 991, /*  215 */
 993, /*  216 */
 994, /*  217 */
 996, /*  218 */
 997, /*  219 */
 999, /*  220 */
1000, /*  221 */
1001, /*  222 */
1003, /*  223 */
1004, /*  224 */
1005, /*  225 */
1006, /*  226 */
1007, /*  227 */
1008, /*  228 */
1009, /*  229 */
1010, /*  230 */
1011, /*  231 */
1012, /*  232 */
1013, /*  233 */
1014, /*  234 */
1015, /*  235 */
1016, /*  236 */
1017, /*  237 */
1017, /*  238 */
1018, /*  239 */
1019, /*  240 */
1019, /*  241 */
1020, /*  242 */
1020, /*  243 */
1021, /*  244 */
1021, /*  245 */
1022, /*  246 */
1022, /*  247 */
1022, /*  248 */
1023, /*  249 */
1023, /*  250 */
1023, /*  251 */
1023, /*  252 */
1023, /*  253 */
1023, /*  254 */
1023, /*  255 */
1024, /*  256 */
1023, /*  257 */
1023, /*  258 */
1023, /*  259 */
1023, /*  260 */
1023, /*  261 */
1023, /*  262 */
1023, /*  263 */
1022, /*  264 */
1022, /*  265 */
1022, /*  266 */
1021, /*  267 */
1021, /*  268 */
1020, /*  269 */
1020, /*  270 */
1019, /*  271 */
1019, /*  272 */
1018, /*  273 */
1017, /*  274 */
1017, /*  275 */
1016, /*  276 */
1015, /*  277 */
1014, /*  278 */
1013, /*  279 */
1012, /*  280 */
1011, /*  281 */
1010, /*  282 */
1009, /*  283 */
1008, /*  284 */
1007, /*  285 */
1006, /*  286 */
1005, /*  287 */
1004, /*  288 */
1003, /*  289 */
1001, /*  290 */
1000, /*  291 */
 999, /*  292 */
 997, /*  293 */
 996, /*  294 */
 994, /*  295 */
 993, /*  296 */
 991, /*  297 */
 990, /*  298 */
 988, /*  299 */
 986, /*  300 */
 985, /*  301 */
 983, /*  302 */
 981, /*  303 */
 979, /*  304 */
 978, /*  305 */
 976, /*  306 */
 974, /*  307 */
 972, /*  308 */
 970, /*  309 */
 968, /*  310 */
 966, /*  311 */
 964, /*  312 */
 962, /*  313 */
 959, /*  314 */
 957, /*  315 */
 955, /*  316 */
 953, /*  317 */
 950, /*  318 */
 948, /*  319 */
 946, /*  320 */
 943, /*  321 */
 941, /*  322 */
 938, /*  323 */
 936, /*  324 */
 933, /*  325 */
 930, /*  326 */
 928, /*  327 */
 925, /*  328 */
 922, /*  329 */
 920, /*  330 */
 917, /*  331 */
 914, /*  332 */
 911, /*  333 */
 908, /*  334 */
 906, /*  335 */
 903, /*  336 */
 900, /*  337 */
 897, /*  338 */
 894, /*  339 */
 890, /*  340 */
 887, /*  341 */
 884, /*  342 */
 881, /*  343 */
 878, /*  344 */
 875, /*  345 */
 871, /*  346 */
 868, /*  347 */
 865, /*  348 */
 861, /*  349 */
 858, /*  350 */
 854, /*  351 */
 851, /*  352 */
 847, /*  353 */
 844, /*  354 */
 840, /*  355 */
 837, /*  356 */
 833, /*  357 */
 829, /*  358 */
 826, /*  359 */
 822, /*  360 */
 818, /*  361 */
 814, /*  362 */
 811, /*  363 */
 807, /*  364 */
 803, /*  365 */
 799, /*  366 */
 795, /*  367 */
 791, /*  368 */
 787, /*  369 */
 783, /*  370 */
 779, /*  371 */
 775, /*  372 */
 771, /*  373 */
 767, /*  374 */
 762, /*  375 */
 758, /*  376 */
 754, /*  377 */
 750, /*  378 */
 745, /*  379 */
 741, /*  380 */
 737, /*  381 */
 732, /*  382 */
 728, /*  383 */
 724, /*  384 */
 719, /*  385 */
 715, /*  386 */
 710, /*  387 */
 706, /*  388 */
 701, /*  389 */
 696, /*  390 */
 692, /*  391 */
 687, /*  392 */
 683, /*  393 */
 678, /*  394 */
 673, /*  395 */
 668, /*  396 */
 664, /*  397 */
 659, /*  398 */
 654, /*  399 */
 649, /*  400 */
 644, /*  401 */
 639, /*  402 */
 634, /*  403 */
 629, /*  404 */
 625, /*  405 */
 620, /*  406 */
 615, /*  407 */
 609, /*  408 */
 604, /*  409 */
 599, /*  410 */
 594, /*  411 */
 589, /*  412 */
 584, /*  413 */
 579, /*  414 */
 574, /*  415 */
 568, /*  416 */
 563, /*  417 */
 558, /*  418 */
 553, /*  419 */
 547, /*  420 */
 542, /*  421 */
 537, /*  422 */
 531, /*  423 */
 526, /*  424 */
 521, /*  425 */
 515, /*  426 */
 510, /*  427 */
 504, /*  428 */
 499, /*  429 */
 493, /*  430 */
 488, /*  431 */
 482, /*  432 */
 477, /*  433 */
 471, /*  434 */
 466, /*  435 */
 460, /*  436 */
 454, /*  437 */
 449, /*  438 */
 443, /*  439 */
 437, /*  440 */
 432, /*  441 */
 426, /*  442 */
 420, /*  443 */
 414, /*  444 */
 409, /*  445 */
 403, /*  446 */
 397, /*  447 */
 391, /*  448 */
 386, /*  449 */
 380, /*  450 */
 374, /*  451 */
 368, /*  452 */
 362, /*  453 */
 356, /*  454 */
 350, /*  455 */
 344, /*  456 */
 339, /*  457 */
 333, /*  458 */
 327, /*  459 */
 321, /*  460 */
 315, /*  461 */
 309, /*  462 */
 303, /*  463 */
 297, /*  464 */
 291, /*  465 */
 285, /*  466 */
 279, /*  467 */
 273, /*  468 */
 267, /*  469 */
 260, /*  470 */
 254, /*  471 */
 248, /*  472 */
 242, /*  473 */
 236, /*  474 */
 230, /*  475 */
 224, /*  476 */
 218, /*  477 */
 212, /*  478 */
 205, /*  479 */
 199, /*  480 */
 193, /*  481 */
 187, /*  482 */
 181, /*  483 */
 175, /*  484 */
 168, /*  485 */
 162, /*  486 */
 156, /*  487 */
 150, /*  488 */
 144, /*  489 */
 137, /*  490 */
 131, /*  491 */
 125, /*  492 */
 119, /*  493 */
 112, /*  494 */
 106, /*  495 */
 100, /*  496 */
  94, /*  497 */
  87, /*  498 */
  81, /*  499 */
  75, /*  500 */
  69, /*  501 */
  62, /*  502 */
  56, /*  503 */
  50, /*  504 */
  43, /*  505 */
  37, /*  506 */
  31, /*  507 */
  25, /*  508 */
  18, /*  509 */
  12, /*  510 */
   6, /*  511 */
   0, /*  512 */
  -7, /*  513 */
 -13, /*  514 */
 -19, /*  515 */
 -26, /*  516 */
 -32, /*  517 */
 -38, /*  518 */
 -44, /*  519 */
 -51, /*  520 */
 -57, /*  521 */
 -63, /*  522 */
 -70, /*  523 */
 -76, /*  524 */
 -82, /*  525 */
 -88, /*  526 */
 -95, /*  527 */
-101, /*  528 */
-107, /*  529 */
-113, /*  530 */
-120, /*  531 */
-126, /*  532 */
-132, /*  533 */
-138, /*  534 */
-145, /*  535 */
-151, /*  536 */
-157, /*  537 */
-163, /*  538 */
-169, /*  539 */
-176, /*  540 */
-182, /*  541 */
-188, /*  542 */
-194, /*  543 */
-200, /*  544 */
-206, /*  545 */
-213, /*  546 */
-219, /*  547 */
-225, /*  548 */
-231, /*  549 */
-237, /*  550 */
-243, /*  551 */
-249, /*  552 */
-255, /*  553 */
-261, /*  554 */
-268, /*  555 */
-274, /*  556 */
-280, /*  557 */
-286, /*  558 */
-292, /*  559 */
-298, /*  560 */
-304, /*  561 */
-310, /*  562 */
-316, /*  563 */
-322, /*  564 */
-328, /*  565 */
-334, /*  566 */
-340, /*  567 */
-345, /*  568 */
-351, /*  569 */
-357, /*  570 */
-363, /*  571 */
-369, /*  572 */
-375, /*  573 */
-381, /*  574 */
-387, /*  575 */
-392, /*  576 */
-398, /*  577 */
-404, /*  578 */
-410, /*  579 */
-415, /*  580 */
-421, /*  581 */
-427, /*  582 */
-433, /*  583 */
-438, /*  584 */
-444, /*  585 */
-450, /*  586 */
-455, /*  587 */
-461, /*  588 */
-467, /*  589 */
-472, /*  590 */
-478, /*  591 */
-483, /*  592 */
-489, /*  593 */
-494, /*  594 */
-500, /*  595 */
-505, /*  596 */
-511, /*  597 */
-516, /*  598 */
-522, /*  599 */
-527, /*  600 */
-532, /*  601 */
-538, /*  602 */
-543, /*  603 */
-548, /*  604 */
-554, /*  605 */
-559, /*  606 */
-564, /*  607 */
-569, /*  608 */
-575, /*  609 */
-580, /*  610 */
-585, /*  611 */
-590, /*  612 */
-595, /*  613 */
-600, /*  614 */
-605, /*  615 */
-610, /*  616 */
-616, /*  617 */
-621, /*  618 */
-626, /*  619 */
-630, /*  620 */
-635, /*  621 */
-640, /*  622 */
-645, /*  623 */
-650, /*  624 */
-655, /*  625 */
-660, /*  626 */
-665, /*  627 */
-669, /*  628 */
-674, /*  629 */
-679, /*  630 */
-684, /*  631 */
-688, /*  632 */
-693, /*  633 */
-697, /*  634 */
-702, /*  635 */
-707, /*  636 */
-711, /*  637 */
-716, /*  638 */
-720, /*  639 */
-725, /*  640 */
-729, /*  641 */
-733, /*  642 */
-738, /*  643 */
-742, /*  644 */
-746, /*  645 */
-751, /*  646 */
-755, /*  647 */
-759, /*  648 */
-763, /*  649 */
-768, /*  650 */
-772, /*  651 */
-776, /*  652 */
-780, /*  653 */
-784, /*  654 */
-788, /*  655 */
-792, /*  656 */
-796, /*  657 */
-800, /*  658 */
-804, /*  659 */
-808, /*  660 */
-812, /*  661 */
-815, /*  662 */
-819, /*  663 */
-823, /*  664 */
-827, /*  665 */
-830, /*  666 */
-834, /*  667 */
-838, /*  668 */
-841, /*  669 */
-845, /*  670 */
-848, /*  671 */
-852, /*  672 */
-855, /*  673 */
-859, /*  674 */
-862, /*  675 */
-866, /*  676 */
-869, /*  677 */
-872, /*  678 */
-876, /*  679 */
-879, /*  680 */
-882, /*  681 */
-885, /*  682 */
-888, /*  683 */
-891, /*  684 */
-895, /*  685 */
-898, /*  686 */
-901, /*  687 */
-904, /*  688 */
-907, /*  689 */
-909, /*  690 */
-912, /*  691 */
-915, /*  692 */
-918, /*  693 */
-921, /*  694 */
-923, /*  695 */
-926, /*  696 */
-929, /*  697 */
-931, /*  698 */
-934, /*  699 */
-937, /*  700 */
-939, /*  701 */
-942, /*  702 */
-944, /*  703 */
-947, /*  704 */
-949, /*  705 */
-951, /*  706 */
-954, /*  707 */
-956, /*  708 */
-958, /*  709 */
-960, /*  710 */
-963, /*  711 */
-965, /*  712 */
-967, /*  713 */
-969, /*  714 */
-971, /*  715 */
-973, /*  716 */
-975, /*  717 */
-977, /*  718 */
-979, /*  719 */
-980, /*  720 */
-982, /*  721 */
-984, /*  722 */
-986, /*  723 */
-987, /*  724 */
-989, /*  725 */
-991, /*  726 */
-992, /*  727 */
-994, /*  728 */
-995, /*  729 */
-997, /*  730 */
-998, /*  731 */
-1000, /*  732 */
-1001, /*  733 */
-1002, /*  734 */
-1004, /*  735 */
-1005, /*  736 */
-1006, /*  737 */
-1007, /*  738 */
-1008, /*  739 */
-1009, /*  740 */
-1010, /*  741 */
-1011, /*  742 */
-1012, /*  743 */
-1013, /*  744 */
-1014, /*  745 */
-1015, /*  746 */
-1016, /*  747 */
-1017, /*  748 */
-1018, /*  749 */
-1018, /*  750 */
-1019, /*  751 */
-1020, /*  752 */
-1020, /*  753 */
-1021, /*  754 */
-1021, /*  755 */
-1022, /*  756 */
-1022, /*  757 */
-1023, /*  758 */
-1023, /*  759 */
-1023, /*  760 */
-1024, /*  761 */
-1024, /*  762 */
-1024, /*  763 */
-1024, /*  764 */
-1024, /*  765 */
-1024, /*  766 */
-1024, /*  767 */
-1024, /*  768 */
-1024, /*  769 */
-1024, /*  770 */
-1024, /*  771 */
-1024, /*  772 */
-1024, /*  773 */
-1024, /*  774 */
-1024, /*  775 */
-1023, /*  776 */
-1023, /*  777 */
-1023, /*  778 */
-1022, /*  779 */
-1022, /*  780 */
-1021, /*  781 */
-1021, /*  782 */
-1020, /*  783 */
-1020, /*  784 */
-1019, /*  785 */
-1018, /*  786 */
-1018, /*  787 */
-1017, /*  788 */
-1016, /*  789 */
-1015, /*  790 */
-1014, /*  791 */
-1013, /*  792 */
-1012, /*  793 */
-1011, /*  794 */
-1010, /*  795 */
-1009, /*  796 */
-1008, /*  797 */
-1007, /*  798 */
-1006, /*  799 */
-1005, /*  800 */
-1004, /*  801 */
-1002, /*  802 */
-1001, /*  803 */
-1000, /*  804 */
-998, /*  805 */
-997, /*  806 */
-995, /*  807 */
-994, /*  808 */
-992, /*  809 */
-991, /*  810 */
-989, /*  811 */
-987, /*  812 */
-986, /*  813 */
-984, /*  814 */
-982, /*  815 */
-980, /*  816 */
-979, /*  817 */
-977, /*  818 */
-975, /*  819 */
-973, /*  820 */
-971, /*  821 */
-969, /*  822 */
-967, /*  823 */
-965, /*  824 */
-963, /*  825 */
-960, /*  826 */
-958, /*  827 */
-956, /*  828 */
-954, /*  829 */
-951, /*  830 */
-949, /*  831 */
-947, /*  832 */
-944, /*  833 */
-942, /*  834 */
-939, /*  835 */
-937, /*  836 */
-934, /*  837 */
-931, /*  838 */
-929, /*  839 */
-926, /*  840 */
-923, /*  841 */
-921, /*  842 */
-918, /*  843 */
-915, /*  844 */
-912, /*  845 */
-909, /*  846 */
-907, /*  847 */
-904, /*  848 */
-901, /*  849 */
-898, /*  850 */
-895, /*  851 */
-891, /*  852 */
-888, /*  853 */
-885, /*  854 */
-882, /*  855 */
-879, /*  856 */
-876, /*  857 */
-872, /*  858 */
-869, /*  859 */
-866, /*  860 */
-862, /*  861 */
-859, /*  862 */
-855, /*  863 */
-852, /*  864 */
-848, /*  865 */
-845, /*  866 */
-841, /*  867 */
-838, /*  868 */
-834, /*  869 */
-830, /*  870 */
-827, /*  871 */
-823, /*  872 */
-819, /*  873 */
-815, /*  874 */
-812, /*  875 */
-808, /*  876 */
-804, /*  877 */
-800, /*  878 */
-796, /*  879 */
-792, /*  880 */
-788, /*  881 */
-784, /*  882 */
-780, /*  883 */
-776, /*  884 */
-772, /*  885 */
-768, /*  886 */
-763, /*  887 */
-759, /*  888 */
-755, /*  889 */
-751, /*  890 */
-746, /*  891 */
-742, /*  892 */
-738, /*  893 */
-733, /*  894 */
-729, /*  895 */
-725, /*  896 */
-720, /*  897 */
-716, /*  898 */
-711, /*  899 */
-707, /*  900 */
-702, /*  901 */
-697, /*  902 */
-693, /*  903 */
-688, /*  904 */
-684, /*  905 */
-679, /*  906 */
-674, /*  907 */
-669, /*  908 */
-665, /*  909 */
-660, /*  910 */
-655, /*  911 */
-650, /*  912 */
-645, /*  913 */
-640, /*  914 */
-635, /*  915 */
-630, /*  916 */
-626, /*  917 */
-621, /*  918 */
-616, /*  919 */
-610, /*  920 */
-605, /*  921 */
-600, /*  922 */
-595, /*  923 */
-590, /*  924 */
-585, /*  925 */
-580, /*  926 */
-575, /*  927 */
-569, /*  928 */
-564, /*  929 */
-559, /*  930 */
-554, /*  931 */
-548, /*  932 */
-543, /*  933 */
-538, /*  934 */
-532, /*  935 */
-527, /*  936 */
-522, /*  937 */
-516, /*  938 */
-511, /*  939 */
-505, /*  940 */
-500, /*  941 */
-494, /*  942 */
-489, /*  943 */
-483, /*  944 */
-478, /*  945 */
-472, /*  946 */
-467, /*  947 */
-461, /*  948 */
-455, /*  949 */
-450, /*  950 */
-444, /*  951 */
-438, /*  952 */
-433, /*  953 */
-427, /*  954 */
-421, /*  955 */
-415, /*  956 */
-410, /*  957 */
-404, /*  958 */
-398, /*  959 */
-392, /*  960 */
-387, /*  961 */
-381, /*  962 */
-375, /*  963 */
-369, /*  964 */
-363, /*  965 */
-357, /*  966 */
-351, /*  967 */
-345, /*  968 */
-340, /*  969 */
-334, /*  970 */
-328, /*  971 */
-322, /*  972 */
-316, /*  973 */
-310, /*  974 */
-304, /*  975 */
-298, /*  976 */
-292, /*  977 */
-286, /*  978 */
-280, /*  979 */
-274, /*  980 */
-268, /*  981 */
-261, /*  982 */
-255, /*  983 */
-249, /*  984 */
-243, /*  985 */
-237, /*  986 */
-231, /*  987 */
-225, /*  988 */
-219, /*  989 */
-213, /*  990 */
-206, /*  991 */
-200, /*  992 */
-194, /*  993 */
-188, /*  994 */
-182, /*  995 */
-176, /*  996 */
-169, /*  997 */
-163, /*  998 */
-157, /*  999 */
-151, /* 1000 */
-145, /* 1001 */
-138, /* 1002 */
-132, /* 1003 */
-126, /* 1004 */
-120, /* 1005 */
-113, /* 1006 */
-107, /* 1007 */
-101, /* 1008 */
 -95, /* 1009 */
 -88, /* 1010 */
 -82, /* 1011 */
 -76, /* 1012 */
 -70, /* 1013 */
 -63, /* 1014 */
 -57, /* 1015 */
 -51, /* 1016 */
 -44, /* 1017 */
 -38, /* 1018 */
 -32, /* 1019 */
 -26, /* 1020 */
 -19, /* 1021 */
 -13, /* 1022 */
  -7}; /* 1023 */

#endif /* SINETABLE_H_ */
