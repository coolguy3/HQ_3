
struct Quad_PID
{
	float target;  		// Ŀ��ֵ
	float current; 		// ��ǰֵ
	float merror;			//��ǰƫ��
	float last_error;	//�ϴ�ƫ��
	float Integrator;	//��ǰ����ֵ(δ����Ki)
	float deriv;			//��ǰ΢��ֵ(δ����Kd)
	float iLimit;			//��������ֵ
	float Kp;					//����ϵ��
	float Ki;					//����ϵ��
	float Kd;					//΢��ϵ��

	float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
	float PID_out;   		//��ǰPID�����
		
};

#define DEFAULT_PID_INTEGRATION_LIMIT  1000.0

void pidInit(struct Quad_PID* pid, const float kp,
             const float ki, const float kd);
float PID_Stand_Update(struct Quad_PID* pid, float measured);
void pidSetIntegralLimit(struct Quad_PID* pid, float limit);
void pidSetError(struct Quad_PID* pid, float err);
void pidReset(struct Quad_PID* pid);
void pidSetTarget(struct Quad_PID* pid, float target);
void pidSetKp(struct Quad_PID* pid, float kp);
void pidSetKi(struct Quad_PID* pid, float ki);
void pidSetKd(struct Quad_PID* pid, float kd);
void pidSetMeasured(struct Quad_PID* pid, float measured);

