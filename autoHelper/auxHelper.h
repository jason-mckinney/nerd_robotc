int aux1 [10];
int aux2 [10];

PID aux1PID;
PID aux2PID;

float aux1SetPoint;
int aux1SensorPort;

float aux2SetPoint;
int aux2SensorPort;

bool aux1HoldRunning = false;
bool aux2HoldRunning = false;

void aux1Stop ();
void aux1HoldStop ();
void aux2Stop ();
void aux2HoldStop ();
void driveAux1 (int speed);
void driveAux2 (int speed);
void aux1Hold (float setPoint, float range);
void aux2Hold (float setPoint, float range);
task taskAux1Hold ();
task taskAux2Hold ();
