struct PID_t {
    float sum_sens,
          lin_sens,
          dif_sens,
          sum,
          prev_val;
};

float update(struct PID_t* PID, float val, float time_elapsed) {
    PID->sum += val * PID->sum_sens;
    float control = PID->sum + val * PID->lin_sens + (val - PID->prev_val) / time_elapsed * PID->dif_sens;
    PID->prev_val = val;
    return control;
}