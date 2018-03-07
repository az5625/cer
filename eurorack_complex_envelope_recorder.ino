/*
  CER - Complex Envelope Recorder

  This device is used to record and playback 0-5V envelopes.
  Creative Commons License
  CER by Pantala Labs is licensed
  under a Creative Commons Attribution 4.0 International License.
  Gibran Curtiss Salomao. MAR/2018 - CC-BY-SA
*/

#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#define DAC_RESOLUTION (8)

//#include <SparkFun_ADXL345.h>
//ADXL345 adxl = ADXL345();

#define       MAXREADINGS           512
#define       us_TRIGGERDEBOUNCE    200000UL
#define       us_DEFAULTINTERVAL    10000UL
#define       us_MININTERVAL        350UL
#define       us_MAXINTERVAL        40000UL
#define       LDRLOWLIMIT           180     //must be higher than minimum reading
#define       LDRHILIMIT            840     //must be lower than maximumreading
#define       ACCELOFFSET           2048
#define       CVINWAVETYPE          5
#define       LDRINWAVETYPE         6


//digital pins
#define       PLAYTRIGGERPIN        2
#define       RECORDFLAGPIN         3
#define       REPEATMODEPIN         4
#define       LEDRECORDINGPIN       6

//analog pins
#define       WAVETYPEPIN           0
#define       ANALOGLDRINPIN        1
#define       STRETCHPIN            2
#define       ENDPLAYPIN            3
#define       L2CSDA                4
#define       L2CSCL                5
#define       STARTPLAYPIN          6
#define       ANALOGCVINPIN         7

unsigned int  potTableCursor = 0;  //{STARTPLAYPIN, ENDPLAYPIN, STRETCHPIN, WAVETYPE};

unsigned int  waveTable[MAXREADINGS];

unsigned int  recEofCursor         = 0;
//unsigned int recEofCv             = 0;
//unsigned int recEofLdr            = 0;
unsigned int  playCursor           = 0;
unsigned int  startPlayCursor      = 0;
unsigned int  endPlayCursor        = MAXREADINGS - 1;
unsigned int  interval             = 0;
unsigned int  waveType             = 0;

boolean       repeatMode            = false;
boolean       flagRecordNew         = false;
boolean       stateRecording        = false;
boolean       flagStopRecording     = false;
boolean       statePlaying          = false;
boolean       flagStopEnvelope      = false;
boolean       flagStartDac          = false;

unsigned long us_lastRecordTime  = 0UL;
unsigned long us_lastPlayTime    = 0UL;
unsigned long us_lastTriggerTime = 0UL;

unsigned long us_now = micros();

unsigned int  oldCVread        = 0;
unsigned int  reading          = 0;
boolean       interpolate      = false;

boolean debug = false;

const PROGMEM unsigned int sinwave[512] =
{
  2048, 2073, 2098, 2123, 2148, 2174, 2199, 2224,
  2249, 2274, 2299, 2324, 2349, 2373, 2398, 2423,
  2448, 2472, 2497, 2521, 2546, 2570, 2594, 2618,
  2643, 2667, 2690, 2714, 2738, 2762, 2785, 2808,
  2832, 2855, 2878, 2901, 2924, 2946, 2969, 2991,
  3013, 3036, 3057, 3079, 3101, 3122, 3144, 3165,
  3186, 3207, 3227, 3248, 3268, 3288, 3308, 3328,
  3347, 3367, 3386, 3405, 3423, 3442, 3460, 3478,
  3496, 3514, 3531, 3548, 3565, 3582, 3599, 3615,
  3631, 3647, 3663, 3678, 3693, 3708, 3722, 3737,
  3751, 3765, 3778, 3792, 3805, 3817, 3830, 3842,
  3854, 3866, 3877, 3888, 3899, 3910, 3920, 3930,
  3940, 3950, 3959, 3968, 3976, 3985, 3993, 4000,
  4008, 4015, 4022, 4028, 4035, 4041, 4046, 4052,
  4057, 4061, 4066, 4070, 4074, 4077, 4081, 4084,
  4086, 4088, 4090, 4092, 4094, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4094, 4092, 4090, 4088,
  4086, 4084, 4081, 4077, 4074, 4070, 4066, 4061,
  4057, 4052, 4046, 4041, 4035, 4028, 4022, 4015,
  4008, 4000, 3993, 3985, 3976, 3968, 3959, 3950,
  3940, 3930, 3920, 3910, 3899, 3888, 3877, 3866,
  3854, 3842, 3830, 3817, 3805, 3792, 3778, 3765,
  3751, 3737, 3722, 3708, 3693, 3678, 3663, 3647,
  3631, 3615, 3599, 3582, 3565, 3548, 3531, 3514,
  3496, 3478, 3460, 3442, 3423, 3405, 3386, 3367,
  3347, 3328, 3308, 3288, 3268, 3248, 3227, 3207,
  3186, 3165, 3144, 3122, 3101, 3079, 3057, 3036,
  3013, 2991, 2969, 2946, 2924, 2901, 2878, 2855,
  2832, 2808, 2785, 2762, 2738, 2714, 2690, 2667,
  2643, 2618, 2594, 2570, 2546, 2521, 2497, 2472,
  2448, 2423, 2398, 2373, 2349, 2324, 2299, 2274,
  2249, 2224, 2199, 2174, 2148, 2123, 2098, 2073,
  2048, 2023, 1998, 1973, 1948, 1922, 1897, 1872,
  1847, 1822, 1797, 1772, 1747, 1723, 1698, 1673,
  1648, 1624, 1599, 1575, 1550, 1526, 1502, 1478,
  1453, 1429, 1406, 1382, 1358, 1334, 1311, 1288,
  1264, 1241, 1218, 1195, 1172, 1150, 1127, 1105,
  1083, 1060, 1039, 1017,  995,  974,  952,  931,
  910,  889,  869,  848,  828,  808,  788,  768,
  749,  729,  710,  691,  673,  654,  636,  618,
  600,  582,  565,  548,  531,  514,  497,  481,
  465,  449,  433,  418,  403,  388,  374,  359,
  345,  331,  318,  304,  291,  279,  266,  254,
  242,  230,  219,  208,  197,  186,  176,  166,
  156,  146,  137,  128,  120,  111,  103,   96,
  88,   81,   74,   68,   61,   55,   50,   44,
  39,   35,   30,   26,   22,   19,   15,   12,
  10,    8,    6,    4,    2,    1,    1,    0,
  0,    0,    1,    1,    2,    4,    6,    8,
  10,   12,   15,   19,   22,   26,   30,   35,
  39,   44,   50,   55,   61,   68,   74,   81,
  88,   96,  103,  111,  120,  128,  137,  146,
  156,  166,  176,  186,  197,  208,  219,  230,
  242,  254,  266,  279,  291,  304,  318,  331,
  345,  359,  374,  388,  403,  418,  433,  449,
  465,  481,  497,  514,  531,  548,  565,  582,
  600,  618,  636,  654,  673,  691,  710,  729,
  749,  768,  788,  808,  828,  848,  869,  889,
  910,  931,  952,  974,  995, 1017, 1039, 1060,
  1083, 1105, 1127, 1150, 1172, 1195, 1218, 1241,
  1264, 1288, 1311, 1334, 1358, 1382, 1406, 1429,
  1453, 1478, 1502, 1526, 1550, 1575, 1599, 1624,
  1648, 1673, 1698, 1723, 1747, 1772, 1797, 1822,
  1847, 1872, 1897, 1922, 1948, 1973, 1998, 2023
};

const PROGMEM unsigned int triwave[512] =
{
  2047, 2063, 2079, 2095, 2111, 2127, 2143, 2159,
  2175, 2191, 2207, 2223, 2239, 2255, 2271, 2287,
  2303, 2319, 2335, 2351, 2367, 2383, 2399, 2415,
  2431, 2447, 2463, 2479, 2495, 2511, 2527, 2543,
  2559, 2575, 2591, 2607, 2623, 2639, 2655, 2671,
  2687, 2703, 2719, 2735, 2751, 2767, 2783, 2799,
  2815, 2831, 2847, 2863, 2879, 2895, 2911, 2927,
  2943, 2959, 2975, 2991, 3007, 3023, 3039, 3055,
  3071, 3087, 3103, 3119, 3135, 3151, 3167, 3183,
  3199, 3215, 3231, 3247, 3263, 3279, 3295, 3311,
  3327, 3343, 3359, 3375, 3391, 3407, 3423, 3439,
  3455, 3471, 3487, 3503, 3519, 3535, 3551, 3567,
  3583, 3599, 3615, 3631, 3647, 3663, 3679, 3695,
  3711, 3727, 3743, 3759, 3775, 3791, 3807, 3823,
  3839, 3855, 3871, 3887, 3903, 3919, 3935, 3951,
  3967, 3983, 3999, 4015, 4031, 4047, 4063, 4079,
  4095, 4079, 4063, 4047, 4031, 4015, 3999, 3983,
  3967, 3951, 3935, 3919, 3903, 3887, 3871, 3855,
  3839, 3823, 3807, 3791, 3775, 3759, 3743, 3727,
  3711, 3695, 3679, 3663, 3647, 3631, 3615, 3599,
  3583, 3567, 3551, 3535, 3519, 3503, 3487, 3471,
  3455, 3439, 3423, 3407, 3391, 3375, 3359, 3343,
  3327, 3311, 3295, 3279, 3263, 3247, 3231, 3215,
  3199, 3183, 3167, 3151, 3135, 3119, 3103, 3087,
  3071, 3055, 3039, 3023, 3007, 2991, 2975, 2959,
  2943, 2927, 2911, 2895, 2879, 2863, 2847, 2831,
  2815, 2799, 2783, 2767, 2751, 2735, 2719, 2703,
  2687, 2671, 2655, 2639, 2623, 2607, 2591, 2575,
  2559, 2543, 2527, 2511, 2495, 2479, 2463, 2447,
  2431, 2415, 2399, 2383, 2367, 2351, 2335, 2319,
  2303, 2287, 2271, 2255, 2239, 2223, 2207, 2191,
  2175, 2159, 2143, 2127, 2111, 2095, 2079, 2063,
  2047, 2031, 2015, 1999, 1983, 1967, 1951, 1935,
  1919, 1903, 1887, 1871, 1855, 1839, 1823, 1807,
  1791, 1775, 1759, 1743, 1727, 1711, 1695, 1679,
  1663, 1647, 1631, 1615, 1599, 1583, 1567, 1551,
  1535, 1519, 1503, 1487, 1471, 1455, 1439, 1423,
  1407, 1391, 1375, 1359, 1343, 1327, 1311, 1295,
  1279, 1263, 1247, 1231, 1215, 1199, 1183, 1167,
  1151, 1135, 1119, 1103, 1087, 1071, 1055, 1039,
  1023, 1007, 991, 975, 959, 943, 927, 911,
  895, 879, 863, 847, 831, 815, 799, 783,
  767, 751, 735, 719, 703, 687, 671, 655,
  639, 623, 607, 591, 575, 559, 543, 527,
  511, 495, 479, 463, 447, 431, 415, 399,
  383, 367, 351, 335, 319, 303, 287, 271,
  255, 239, 223, 207, 191, 175, 159, 143,
  127, 111, 95, 79, 63, 47, 31, 15,
  0, 15, 31, 47, 63, 79, 95, 111,
  127, 143, 159, 175, 191, 207, 223, 239,
  255, 271, 287, 303, 319, 335, 351, 367,
  383, 399, 415, 431, 447, 463, 479, 495,
  511, 527, 543, 559, 575, 591, 607, 623,
  639, 655, 671, 687, 703, 719, 735, 751,
  767, 783, 799, 815, 831, 847, 863, 879,
  895, 911, 927, 943, 959, 975, 991, 1007,
  1023, 1039, 1055, 1071, 1087, 1103, 1119, 1135,
  1151, 1167, 1183, 1199, 1215, 1231, 1247, 1263,
  1279, 1295, 1311, 1327, 1343, 1359, 1375, 1391,
  1407, 1423, 1439, 1455, 1471, 1487, 1503, 1519,
  1535, 1551, 1567, 1583, 1599, 1615, 1631, 1647,
  1663, 1679, 1695, 1711, 1727, 1743, 1759, 1775,
  1791, 1807, 1823, 1839, 1855, 1871, 1887, 1903,
  1919, 1935, 1951, 1967, 1983, 1999, 2015, 2031,
};

const PROGMEM unsigned int rampwave[512] =
{
  2047, 2063, 2079, 2095, 2111, 2127, 2143, 2159,
  2175, 2191, 2207, 2223, 2239, 2255, 2271, 2287,
  2303, 2319, 2335, 2351, 2367, 2383, 2399, 2415,
  2431, 2447, 2463, 2479, 2495, 2511, 2527, 2543,
  2559, 2575, 2591, 2607, 2623, 2639, 2655, 2671,
  2687, 2703, 2719, 2735, 2751, 2767, 2783, 2799,
  2815, 2831, 2847, 2863, 2879, 2895, 2911, 2927,
  2943, 2959, 2975, 2991, 3007, 3023, 3039, 3055,
  3071, 3087, 3103, 3119, 3135, 3151, 3167, 3183,
  3199, 3215, 3231, 3247, 3263, 3279, 3295, 3311,
  3327, 3343, 3359, 3375, 3391, 3407, 3423, 3439,
  3455, 3471, 3487, 3503, 3519, 3535, 3551, 3567,
  3583, 3599, 3615, 3631, 3647, 3663, 3679, 3695,
  3711, 3727, 3743, 3759, 3775, 3791, 3807, 3823,
  3839, 3855, 3871, 3887, 3903, 3919, 3935, 3951,
  3967, 3983, 3999, 4015, 4031, 4047, 4063, 4079,
  0, 16, 32, 48, 64, 80, 96, 112,
  128, 144, 160, 176, 192, 208, 224, 240,
  256, 272, 288, 304, 320, 336, 352, 368,
  384, 400, 416, 432, 448, 464, 480, 496,
  512, 528, 544, 560, 576, 592, 608, 624,
  640, 656, 672, 688, 704, 720, 736, 752,
  768, 784, 800, 816, 832, 848, 864, 880,
  896, 912, 928, 944, 960, 976, 992, 1008,
  1024, 1040, 1056, 1072, 1088, 1104, 1120, 1136,
  1152, 1168, 1184, 1200, 1216, 1232, 1248, 1264,
  1280, 1296, 1312, 1328, 1344, 1360, 1376, 1392,
  1408, 1424, 1440, 1456, 1472, 1488, 1504, 1520,
  1536, 1552, 1568, 1584, 1600, 1616, 1632, 1648,
  1664, 1680, 1696, 1712, 1728, 1744, 1760, 1776,
  1792, 1808, 1824, 1840, 1856, 1872, 1888, 1904,
  1920, 1936, 1952, 1968, 1984, 2000, 2016, 2032,
  2048, 2064, 2080, 2096, 2112, 2128, 2144, 2160,
  2176, 2192, 2208, 2224, 2240, 2256, 2272, 2288,
  2304, 2320, 2336, 2352, 2368, 2384, 2400, 2416,
  2432, 2448, 2464, 2480, 2496, 2512, 2528, 2544,
  2560, 2576, 2592, 2608, 2624, 2640, 2656, 2672,
  2688, 2704, 2720, 2736, 2752, 2768, 2784, 2800,
  2816, 2832, 2848, 2864, 2880, 2896, 2912, 2928,
  2944, 2960, 2976, 2992, 3008, 3024, 3040, 3056,
  3072, 3088, 3104, 3120, 3136, 3152, 3168, 3184,
  3200, 3216, 3232, 3248, 3264, 3280, 3296, 3312,
  3328, 3344, 3360, 3376, 3392, 3408, 3424, 3440,
  3456, 3472, 3488, 3504, 3520, 3536, 3552, 3568,
  3584, 3600, 3616, 3632, 3648, 3664, 3680, 3696,
  3712, 3728, 3744, 3760, 3776, 3792, 3808, 3824,
  3840, 3856, 3872, 3888, 3904, 3920, 3936, 3952,
  3968, 3984, 4000, 4016, 4032, 4048, 4064, 4080,
  0, 16, 32, 48, 64, 80, 96, 112,
  128, 144, 160, 176, 192, 208, 224, 240,
  256, 272, 288, 304, 320, 336, 352, 368,
  384, 400, 416, 432, 448, 464, 480, 496,
  512, 528, 544, 560, 576, 592, 608, 624,
  640, 656, 672, 688, 704, 720, 736, 752,
  768, 784, 800, 816, 832, 848, 864, 880,
  896, 912, 928, 944, 960, 976, 992, 1008,
  1024, 1040, 1056, 1072, 1088, 1104, 1120, 1136,
  1152, 1168, 1184, 1200, 1216, 1232, 1248, 1264,
  1280, 1296, 1312, 1328, 1344, 1360, 1376, 1392,
  1408, 1424, 1440, 1456, 1472, 1488, 1504, 1520,
  1536, 1552, 1568, 1584, 1600, 1616, 1632, 1648,
  1664, 1680, 1696, 1712, 1728, 1744, 1760, 1776,
  1792, 1808, 1824, 1840, 1856, 1872, 1888, 1904,
  1920, 1936, 1952, 1968, 1984, 2000, 2016, 2032,
};

const PROGMEM unsigned int sawwave[512] =
{
  2047, 2031, 2015, 1999, 1983, 1967, 1951, 1935,
  1919, 1903, 1887, 1871, 1855, 1839, 1823, 1807,
  1791, 1775, 1759, 1743, 1727, 1711, 1695, 1679,
  1663, 1647, 1631, 1615, 1599, 1583, 1567, 1551,
  1535, 1519, 1503, 1487, 1471, 1455, 1439, 1423,
  1407, 1391, 1375, 1359, 1343, 1327, 1311, 1295,
  1279, 1263, 1247, 1231, 1215, 1199, 1183, 1167,
  1151, 1135, 1119, 1103, 1087, 1071, 1055, 1039,
  1023, 1007, 991, 975, 959, 943, 927, 911,
  895, 879, 863, 847, 831, 815, 799, 783,
  767, 751, 735, 719, 703, 687, 671, 655,
  639, 623, 607, 591, 575, 559, 543, 527,
  511, 495, 479, 463, 447, 431, 415, 399,
  383, 367, 351, 335, 319, 303, 287, 271,
  255, 239, 223, 207, 191, 175, 159, 143,
  127, 111, 95, 79, 63, 47, 31, 15,
  4095, 4079, 4063, 4047, 4031, 4015, 3999, 3983,
  3967, 3951, 3935, 3919, 3903, 3887, 3871, 3855,
  3839, 3823, 3807, 3791, 3775, 3759, 3743, 3727,
  3711, 3695, 3679, 3663, 3647, 3631, 3615, 3599,
  3583, 3567, 3551, 3535, 3519, 3503, 3487, 3471,
  3455, 3439, 3423, 3407, 3391, 3375, 3359, 3343,
  3327, 3311, 3295, 3279, 3263, 3247, 3231, 3215,
  3199, 3183, 3167, 3151, 3135, 3119, 3103, 3087,
  3071, 3055, 3039, 3023, 3007, 2991, 2975, 2959,
  2943, 2927, 2911, 2895, 2879, 2863, 2847, 2831,
  2815, 2799, 2783, 2767, 2751, 2735, 2719, 2703,
  2687, 2671, 2655, 2639, 2623, 2607, 2591, 2575,
  2559, 2543, 2527, 2511, 2495, 2479, 2463, 2447,
  2431, 2415, 2399, 2383, 2367, 2351, 2335, 2319,
  2303, 2287, 2271, 2255, 2239, 2223, 2207, 2191,
  2175, 2159, 2143, 2127, 2111, 2095, 2079, 2063,
  2047, 2031, 2015, 1999, 1983, 1967, 1951, 1935,
  1919, 1903, 1887, 1871, 1855, 1839, 1823, 1807,
  1791, 1775, 1759, 1743, 1727, 1711, 1695, 1679,
  1663, 1647, 1631, 1615, 1599, 1583, 1567, 1551,
  1535, 1519, 1503, 1487, 1471, 1455, 1439, 1423,
  1407, 1391, 1375, 1359, 1343, 1327, 1311, 1295,
  1279, 1263, 1247, 1231, 1215, 1199, 1183, 1167,
  1151, 1135, 1119, 1103, 1087, 1071, 1055, 1039,
  1023, 1007, 991, 975, 959, 943, 927, 911,
  895, 879, 863, 847, 831, 815, 799, 783,
  767, 751, 735, 719, 703, 687, 671, 655,
  639, 623, 607, 591, 575, 559, 543, 527,
  511, 495, 479, 463, 447, 431, 415, 399,
  383, 367, 351, 335, 319, 303, 287, 271,
  255, 239, 223, 207, 191, 175, 159, 143,
  127, 111, 95, 79, 63, 47, 31, 15,
  4095, 4079, 4063, 4047, 4031, 4015, 3999, 3983,
  3967, 3951, 3935, 3919, 3903, 3887, 3871, 3855,
  3839, 3823, 3807, 3791, 3775, 3759, 3743, 3727,
  3711, 3695, 3679, 3663, 3647, 3631, 3615, 3599,
  3583, 3567, 3551, 3535, 3519, 3503, 3487, 3471,
  3455, 3439, 3423, 3407, 3391, 3375, 3359, 3343,
  3327, 3311, 3295, 3279, 3263, 3247, 3231, 3215,
  3199, 3183, 3167, 3151, 3135, 3119, 3103, 3087,
  3071, 3055, 3039, 3023, 3007, 2991, 2975, 2959,
  2943, 2927, 2911, 2895, 2879, 2863, 2847, 2831,
  2815, 2799, 2783, 2767, 2751, 2735, 2719, 2703,
  2687, 2671, 2655, 2639, 2623, 2607, 2591, 2575,
  2559, 2543, 2527, 2511, 2495, 2479, 2463, 2447,
  2431, 2415, 2399, 2383, 2367, 2351, 2335, 2319,
  2303, 2287, 2271, 2255, 2239, 2223, 2207, 2191,
  2175, 2159, 2143, 2127, 2111, 2095, 2079, 2063,
};

const PROGMEM unsigned int sqrwave[512] =
{
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

void setup()
{
  pinMode(PLAYTRIGGERPIN,   INPUT);
  attachInterrupt(digitalPinToInterrupt(PLAYTRIGGERPIN), int_externalTrigger,   RISING);
  pinMode(RECORDFLAGPIN,    INPUT);
  pinMode(REPEATMODEPIN,    INPUT);
  pinMode(LEDRECORDINGPIN,  OUTPUT);

  if (debug) {
    Serial.begin(9600);
  }
  dac.begin(0x62);
  setDac(0);
}

void loadWavetable(int index) {
  switch (index) {
    case 0 :
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sqrwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 1 :
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(triwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 2:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(rampwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 3:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sawwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 4:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sinwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 5:
    case 6:
      //cv or ldr in
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = 0;
      }
      break;
  }
}

void int_externalTrigger() {
  if ( us_now > (us_lastTriggerTime + us_TRIGGERDEBOUNCE) ) {
    if ( (waveType == CVINWAVETYPE) || (waveType == LDRINWAVETYPE) ) {
      us_lastTriggerTime = us_now;
      //first external trigger to start recording
      if (flagRecordNew) {
        flagRecordNew         = false;
        stateRecording        = true;
        flagStopRecording     = false;
        statePlaying          = false;
        flagStopEnvelope      = false;
        flagStartDac          = true;
        recEofCursor          = 0;
        digitalWrite(LEDRECORDINGPIN, HIGH);
        return;
      }
      //second external trigger to end recording
      if (stateRecording) {
        stateRecording        = false;
        flagStopEnvelope      = false;
        flagStopRecording     = true;
        if (repeatMode) {
          statePlaying        = true;
        }
        flagStopEnvelope      = false;
        interpolate           = true;
        return;
      }
    }
    //else , just play recorded envelope
    statePlaying            = true;
    flagStopEnvelope        = false;
    playCursor              = startPlayCursor;
    interpolate             = true;
  }
}

void loop() {
  us_now = micros();

  //read user inputs
  repeatMode = digitalRead(REPEATMODEPIN);

  //read only one pot / loop
  switch (potTableCursor) {
    case 0:
      startPlayCursor = analogRead(STARTPLAYPIN);
      //prevents not enough voltage on pot
      if (startPlayCursor > 1018) {
        startPlayCursor = 1023;
      }
      startPlayCursor = map(startPlayCursor, 1023, 0, endPlayCursor, 0);
      break;
    case 1:
      endPlayCursor = analogRead(ENDPLAYPIN);
      //prevents not enough voltage on pot
      if (endPlayCursor > 1018) {
        endPlayCursor = 1023;
      }
      endPlayCursor = map(endPlayCursor, 1023, 0, startPlayCursor, recEofCursor);
      break;
    case 2:
      reading = analogRead(STRETCHPIN);
      //prevents not enough voltage on pot
      if (reading > 1018) {
        reading = 1023;
      }
      interval = map(reading, 0, 1023, us_MININTERVAL, us_MAXINTERVAL);
      break;
    case 3:
      reading = analogRead(WAVETYPEPIN);
      reading = map(reading, 0, 1023, 0, 7);
      reading = constrain(reading, 0, 6);
      if (reading != waveType) {
        waveType = reading;
        loadWavetable(waveType);
      }
      break;
  }

  //flags next trigger to record new envelope
  if ( (digitalRead(RECORDFLAGPIN) == HIGH) && ( us_now > (us_lastTriggerTime + us_TRIGGERDEBOUNCE)) ) {
    us_lastTriggerTime = us_now;
    flagRecordNew   = true;
  }

  //stop recording
  if (flagStopRecording) {
    digitalWrite(LEDRECORDINGPIN, LOW);
    flagStopRecording   = false;
    stateRecording      = false;
    playCursor          = startPlayCursor;
    recEofCursor        = recEofCursor - 1;

    //    if (waveType==CVINWAVETYPE) {
    //      for (int i = 0; i <= recEofCursor; i++) {
    //        cvTable[i] = waveTable[i];
    //      }
    //      recEofCv  = recEofCursor;
    //    } else if (waveType==LDRINWAVETYPE) {
    //      for (int i = 0; i <= recEofCursor; i++) {
    //        cvTable[i] = waveTable[i];
    //      }
    //      recEofLdr = recEofCursor;
    //    }

    if (repeatMode) {
      statePlaying = true;
    } else {
      statePlaying = false;
      setDac(0);
    }
  }

  //update recording
  //starts dac if needed
  //  if ( stateRecording && flagStartDac && (recEofCursor == 0) ) {
  //    start_dac();
  //  }
  //if still recording condition and its time to save new sample

  if ( stateRecording && ( micros() >= (us_lastRecordTime + interval) ) ) {
    us_lastRecordTime = micros();                       //save this time
    waveTable[recEofCursor] = readSourceVoltage(recEofCursor, waveType);  //read source voltage from waveType option
    setDac(waveTable[recEofCursor]);                    //send voltage on DAC
    recEofCursor++;                                     //set next rec position
    if (recEofCursor >= MAXREADINGS) {                  //overflow?
      endPlayCursor = recEofCursor;
      flagStopRecording = true;                         //think about into crate a button to an ENDLESS recording !!!!!!!!!!!!!!!
    }
  }

  //update playing
  //if still playing condition and its time to play new sample
  if ( statePlaying && ( micros() >= (us_lastPlayTime + interval) ) ) {
    us_lastPlayTime = micros();                     //save this time
    setDac(waveTable[playCursor]);                  //send voltage on DAC
    playCursor++;                                   //set next play position
    interpolate     = true;                         //flags need interpolation
    if (playCursor > endPlayCursor) {              //EOF ?
      playCursor = startPlayCursor;
      if (!repeatMode) {                            //dont rewind
        statePlaying = false;                       //kill transmission
        setDac(0);
      }
    }
  }

  //interpolate voltage
  //if still playing condition and its time to interpolate sample
  if ( (waveType == CVINWAVETYPE) || (waveType == LDRINWAVETYPE)
       && statePlaying
       && interpolate
       && (playCursor < endPlayCursor)
       && ( micros() >= (us_lastPlayTime + (interval / 2)) )
     ) {
    interpolate = false;
    setDac((waveTable[playCursor] + waveTable[playCursor - 1]) / 2); //send interpolated voltage on DAC
  }

  //change reading pot time
  potTableCursor++;
  if (potTableCursor > 3) {
    potTableCursor = 0;
  }
}

//set voltage on DAC
void setDac(int voltage) {
  dac.setVoltage(voltage, false);
}

//read source voltage and keep it between 0 - 1023
int readSourceVoltage(int arrayPosition, int mode) {
  int thisCVread;
  //--------------------------------------------------
  //accelerometer soften and scale Y reading
  //  int x, y, z;
  //  adxl.readAccel(&x, &y, &z);                             //read
  //  thisCVread = ACCELOFFSET + (y * 15);                    //scale

  if ( (mode == CVINWAVETYPE) || (mode == LDRINWAVETYPE) ) {
    if (mode == CVINWAVETYPE) {
      //synthesizer CV in on A(0) , no smootheness
      thisCVread = 4 * analogRead(ANALOGCVINPIN);
    } else if (mode == LDRINWAVETYPE) {
      //--------------------------------------------------
      //LDR soften, scale and invert readings
      thisCVread = analogRead(ANALOGLDRINPIN);                          //read
      thisCVread = constrain(thisCVread, LDRLOWLIMIT, LDRHILIMIT);      //constrain
      thisCVread = map(thisCVread, LDRLOWLIMIT, LDRHILIMIT, 0, 4095);   //scale
      if (arrayPosition == 0) {                                         //set old value
        oldCVread = thisCVread;
      } else {
        oldCVread = waveTable[arrayPosition - 1];
      }
    }
    return thisCVread;
  }
  return 0;
}

void start_dac() {
  //  adxl.powerOn();
  //  adxl.setRangeSetting(8);            // range settings 2g, 4g, 8g or 16g, Higher Values = Wider Range / Lower Values =  More Sensitivity
  //  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to Default: Set to 1 // SPI pins ATMega328: 11, 12 and 13
  //  adxl.setActivityXYZ(0, 1, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  //  adxl.setActivityThreshold(135);     // 62.5mg unit // Inactivity thresholds (0-255), Higher Values = smooth, Lower Values = crispy
  //  adxl.setInactivityXYZ(0, 1, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  //  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  //  adxl.setTimeInactivity(5);          // How many seconds of no activity is inactive?
  //  adxl.setTapDetectionOnXYZ(0, 0, 0); // No taps
}

