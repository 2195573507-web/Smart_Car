#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float current_target;
    float max_accel;
} Ramp_Profile_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;
    float deadband;
    float integral;
    float previous_error;
    float filtered_feedback;
    float previous_filtered_feedback;
    float feedback_alpha;
    float output;
    unsigned char initialized;
} pid_controller_t;

typedef pid_controller_t PID_Controller_t;

float Ramp_Update(Ramp_Profile_t *ramp, float final_target, float dt_seconds);
void Ramp_Update_Max_Accel(Ramp_Profile_t *ramp, float max_accel);
void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd,
                         float max_out, float max_iout, float deadband);
void PID_Update_Gains(PID_Controller_t *pid, float kp, float ki, float kd);
void pid_controller_reset(pid_controller_t *pid);
float pid_controller_step_with_feedforward(pid_controller_t *pid,
                                            float target, float actual,
                                            float dt_seconds);
float pid_controller_step(pid_controller_t *pid, float target, float actual,
                          float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
