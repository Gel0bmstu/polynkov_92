struct sensor_t {
    float scale;
    float bias;
};

float getval(struct sensor_t* s, char datah, char datal) {
    return (int16_t)((datah << 8 ) | datal) * s->scale - s->bias;
}

float l2g_x_c[] = {-3.44833, 0.0738886, 2.90239E-5, 3.29143E-5};

float get_polynom_zero_bias(float temp, float c[3]) {
    return c[0] + c[1] * temp + c[2] * temp * temp + c[3] * temp * temp * temp;
}