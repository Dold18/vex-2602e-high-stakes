/*vex-vision-config:begin*/
#include "botSetup/vex.h"
vex::vision::signature SIG_1 = vex::vision::signature (1, 9699, 10963, 10331, -617, 97, -260, 2.5, 0);
vex::vision::signature SIG_2 = vex::vision::signature (2, -3817, -3107, -3462, 6629, 8389, 7509, 2.5, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision frontEye = vex::vision (vex::PORT9, 50, SIG_1, SIG_2);
/*vex-vision-config:end*/