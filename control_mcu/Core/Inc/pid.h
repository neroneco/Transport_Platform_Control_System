
#ifndef INC_PID_H_
#define INC_PID_H_

float PD( float eF, float D_e, float P, float D, float dt ) {

    float u = (P*eF + D*D_e);

    return u;
}

float PI( float eS, float u_I[2], float P, float I, float dt ) {

    u_I[0] = u_I[1] + I*dt*eS;
    u_I[1] = u_I[0];

    static float u;
    u = (P*eS + u_I[0]);

    return u;
}

float PD_platform( float eF, float D_eF, float P, float D, float dt ) {

    float u = (P*eF + D*D_eF);

    return u;
}

float PI_cart( float eC, float P, float I, float dt ) {

    static float I_e[2];
    I_e[0] = I_e[1] + I*dt*eC;
    if ( I_e[0] > 20.0 ) {
        I_e[0] = 20.0;
    } else if ( I_e[0] < -20.0 ) {
        I_e[0] = -20.0;
    }

    static int latch;
    if ( (eC > -1.5) && (eC < 1.5) ) {
        I_e[0] = 0.0;
    }
//        if (latch) {
//            I_e[0] = 0.0;
//            //P = 0.0;
//        } else if ( (eC > -1.0) && (eC < 1.0) ) {
//            latch = 1;
//        }

    I_e[1] = I_e[0];

    float u = (P*eC + I_e[0]);

    return u;
}

#endif /* INC_PID_H_ */

