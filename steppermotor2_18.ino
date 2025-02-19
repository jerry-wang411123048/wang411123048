#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define DIR_PIN_1  27
#define STEP_PIN_1 26        
#define ENABLE_PIN_1 25       
#define RX_PIN_1 13          
#define TX_PIN_1 12          
#define STALLGUARD_PIN_1 32

#define DIR_PIN_2 18
#define STEP_PIN_2 19        
#define ENABLE_PIN_2 21       
#define RX_PIN_2 16          
#define TX_PIN_2 17          
#define STALLGUARD_PIN_2 23

#define X_PIN  35
#define SW_PIN 4

bool canMoveRight = true;  // 允許向右移動
bool canMoveLeft = true;
//motormove_1
/*
bool arrive_a = false;           
bool arrive_b = false;
float a_move = 10; //cm
float b_move = 15.6; //cm
int a = a_move*16*200;
int b = b_move*16*200;
*/

//motormove_2

float position[]={7.5,15.7,30.2,44.8};                //輸入目標位置cm
int numPositions = sizeof(position) / sizeof(position[0]);    
bool move = false;
int k ;

bool positionset = false;


//int32_t move_to_step = 3200*5;            
int32_t set_velocity = 3200*3;            
int32_t set_accel = 3200*2;
int32_t set_current = 600;
int32_t set_stall = 30;    
int32_t set_tcools = 500;  
uint16_t motor_microsteps = 16;
uint8_t  TPWMTHRS = 120;


bool pwm_autograd = true;
bool pwm_autoscale = true;
bool en_spreadCycle = false;

uint8_t pwm_reg = 4;
uint8_t pwm_ofs = 40;
uint8_t pwm_grad = 40;
uint8_t hstrt = 5;
uint8_t hend = 3;


bool stalled_motor1 = false;
bool motor_moving1 = false;
bool motor_moving2 = false;

bool motor_state = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
TMC2209Stepper driver1(&Serial1, 0.11f, 0);

FastAccelStepper *stepper2 = NULL;
TMC2209Stepper driver2(&Serial2, 0.11f, 0);


void IRAM_ATTR stalled_position1() {
  stalled_motor1 = true;
}





void setup() {
  Serial.begin(115200);

  Serial1.begin(115200, SERIAL_8N1, RX_PIN_1, TX_PIN_1); 
  Serial2.begin(115200, SERIAL_8N1, RX_PIN_2, TX_PIN_2); 


  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(STALLGUARD_PIN_1, INPUT);
  pinMode(SW_PIN,INPUT_PULLUP);
  
 
  attachInterrupt(digitalPinToInterrupt(STALLGUARD_PIN_1), stalled_position1, RISING);
  
  
 

driver1.begin();   
                   //motor1
 driver1.intpol(false);               
 driver1.toff(4);                        
 driver1.blank_time(24);                
 driver1.I_scale_analog(false);         
 driver1.internal_Rsense(false);       
 driver1.mstep_reg_select(true);       
 driver1.microsteps(motor_microsteps);   

 driver1.shaft(true);  
 driver1.TPWMTHRS(TPWMTHRS);  
 driver1.en_spreadCycle(en_spreadCycle);                  
 driver1.semin(0);                                     
 driver1.VACTUAL(0);  
 driver1.pdn_disable(true);                   
 driver1.rms_current(set_current);
 driver1.SGTHRS(set_stall);
 driver1.TCOOLTHRS(set_tcools);

 driver1.pwm_autoscale(pwm_autoscale); 
 driver1.pwm_autograd(pwm_autograd);
 driver1.pwm_freq(1);
 driver1.pwm_reg(pwm_reg);
 driver1.pwm_ofs(pwm_ofs);
 driver1.pwm_grad(pwm_grad);
  
 driver1.hstrt(hstrt);
 driver1.hend(hend);


  engine.init();

  stepper1 = engine.stepperConnectToPin(STEP_PIN_1);
  stepper1->setDirectionPin(DIR_PIN_1);
  stepper1->setEnablePin(ENABLE_PIN_1);
  stepper1->setAutoEnable(true); 
  stepper1->setSpeedInHz(set_velocity);
 stepper1->setAcceleration(set_accel);     
        //motor1
 driver2.begin();  
 driver2.intpol(false);               //motor2
 driver2.toff(4);                        
 driver2.blank_time(24);                
 driver2.I_scale_analog(false);         
 driver2.internal_Rsense(false);       
 driver2.mstep_reg_select(true);       
 driver2.microsteps(motor_microsteps);   

 driver2.TPWMTHRS(TPWMTHRS);  
 driver2.en_spreadCycle(en_spreadCycle);                  
 driver2.semin(0);                                     
 driver2.VACTUAL(0);  
 driver2.pdn_disable(true);                   
 driver2.rms_current(set_current);
 driver2.SGTHRS(set_stall);
 driver2.TCOOLTHRS(set_tcools);
 driver2.shaft(true);  

 driver2.pwm_autoscale(pwm_autoscale); 
 driver2.pwm_autograd(pwm_autograd);
 driver2.pwm_freq(1);
 driver2.pwm_reg(pwm_reg);
 driver2.pwm_ofs(pwm_ofs);
 driver2.pwm_grad(pwm_grad);
  
 driver2.hstrt(hstrt);
 driver2.hend(hend);                


  stepper2 = engine.stepperConnectToPin(STEP_PIN_2);
  stepper2->setDirectionPin(DIR_PIN_2);
  stepper2->setEnablePin(ENABLE_PIN_2);
  stepper2->setAutoEnable(true); 
  stepper2->setSpeedInHz(set_velocity);
stepper2->setAcceleration(set_accel*3);         //motor2
 

  Serial.println("Setup 1");
 
  

  // 創建控制任務
  xTaskCreatePinnedToCore(MotorTask1, "MotorTask1", 1024 * 4, NULL, 3, NULL, 0);
  
 

  delay(5000);  // 給任務足夠的初始化時間

  
}

void loop() {
  // Add your Wi-Fi code here if required. The motor control will be done in the other task and core.

  while (stepper1->isRunning() == true) {

    
    //Serial.print(" TSTEP1 : ");
    //Serial.println(driver1.TSTEP()*1.0);
    //Serial.print(" TSTEP2 : ");
    //Serial.println(driver2.TSTEP());

    //Serial.print(" OFS :");
    //Serial.print(driver.pwm_ofs_auto());
    
    // Serial.print(" GRAD :");
    //Serial.print(driver.pwm_grad_auto());

    //Serial.print(" scale :");
    //Serial.print(driver.pwm_scale_auto());

    //Serial.print(" sum :");
    //Serial.print(driver.pwm_scale_sum());

    //Serial.print(" PWM_SCALE :");
    //Serial.print(driver.PWM_SCALE());

    //Serial.print(" PWM_AUTO :");
    //Serial.print(driver.PWM_AUTO());

    //uint16_t cs_actual =driver1.cs_actual();
    //Serial.print(" CS_ACTUAL: ");
    //Serial.print(cs_actual);

    //Serial.print(" current :");
    //Serial.print(driver.rms_current());
    
    //Serial.print(" SG_RESULT: ");
    //Serial.println(driver1.SG_RESULT());
   
    
    
    Serial.print(" position1(cm) : ");
    Serial.println(stepper1->getCurrentPosition()/3200.0,3);

    //Serial.print(" position2 : ");
    //Serial.println(stepper2->getCurrentPosition());



    //uint32_t drv_status = driver1.DRV_STATUS();  
  
    //if (drv_status & (1 << 30)) {  
    // Serial.println("StealthChop mode is enabled.");
    //} else {
    // Serial.println("SpreadCycle mode is enabled.");}

  delay(50);  // 每秒檢查一次
  }
  while (stepper2->isRunning() == true) {

    
    //Serial.print(" TSTEP2 : ");
    // Serial.println(driver2.TSTEP()*1.0);

    //Serial.print(" OFS :");
    //Serial.print(driver.pwm_ofs_auto());
    
    // Serial.print(" GRAD :");
    //Serial.print(driver.pwm_grad_auto());

    //Serial.print(" scale :");
    //Serial.print(driver.pwm_scale_auto());

    //Serial.print(" sum :");
    //Serial.print(driver.pwm_scale_sum());

    //Serial.print(" PWM_SCALE :");
    //Serial.print(driver.PWM_SCALE());

    //Serial.print(" PWM_AUTO :");
    //Serial.print(driver.PWM_AUTO());

    //uint16_t cs_actual =driver1.cs_actual();
    //Serial.print(" CS_ACTUAL: ");
    //Serial.print(cs_actual);

    //Serial.print(" current :");
    //Serial.print(driver.rms_current());
    
    //Serial.print(" SG_RESULT: ");
    //Serial.println(driver.SG_RESULT());
   
    
    
    //Serial.print(" position1 : ");
    //Serial.print(stepper1->getCurrentPosition());

    Serial.print(" position2 : ");
    Serial.println(stepper2->getCurrentPosition());



    // uint32_t drv_status2 = driver2.DRV_STATUS();  
  
    //if (drv_status2 & (1 << 30)) {  
    //Serial.println("StealthChop mode is enabled.");
    //} else {
    // Serial.println("SpreadCycle mode is enabled.");}

  delay(50);  // 每秒檢查一次
  }
}





void MotorTask1(void *pvParameters) {

  
  while (!positionset){
    
    delay(5000);
    stepper1->setSpeedInHz(3200);
    stepper1->move(-500000);
    while(stepper1->isRunning() == true){
      if(stalled_motor1 == true){
        stepper1->forceStop();
        stalled_motor1 = false;
        delay(2000);
       break;
      }
      delay(10);
    }
    for (int i = 0; i < numPositions; i++) {
    position[i]= position[i]*16 * 200;  // 每個元素都乘以 16*200
    }
    Serial.println("Position setted");
    delay(2000); 
    positionset = true;

  }
  while(true){

  while (!motor_state){
    int joyx = analogRead(X_PIN);
    if(joyx > 2048 + 200 && canMoveRight){
    stepper2->move(5);
      while(stepper2->isRunning()){
        delay(10);
      }
      canMoveLeft = false; 
      canMoveRight = true;
    }
    else if(joyx < 2048 - 200 && canMoveLeft){
    stepper2->move(-5);
    while(stepper2->isRunning()){
        delay(10);
      }
    canMoveLeft = true; 
    canMoveRight = false;
    }
    else if(joyx >= 2048 - 200 && joyx <= 2048 + 200) {
    canMoveRight = true;
    canMoveLeft = true;
    }
    bool motor_run = digitalRead(SW_PIN);
    if (!motor_run){
    motor_state = true;
    break;
    }
    delay(10);
  }


  while (motor_state){ 
  k = 0;
  Serial.println("Start");
  delay(1000);  
  
  stepper1->setCurrentPosition(0);
  stepper2->setCurrentPosition(0);
  stepper1->setSpeedInHz(set_velocity);
  while(k < numPositions){
  stepper1->moveTo(position[k]);
    while(stepper1->isRunning()){
      delay(10);
    }
    move = true;
    delay(300);
    while(move){
      stepper2->moveTo(1600);
      while(stepper2->isRunning()){
        delay(10);
      }
    delay(300);
      stepper2->moveTo(0);
      while(stepper2->isRunning()){
      delay(10);
      }
     delay(300); 
    move = false;
    } 
    k++;
  }
  delay(1000);
  if(k == numPositions){
    stepper1->moveTo(0);
    while(stepper1->isRunning()){
      delay(10);
    }   
    Serial.println("End");
  }
  motor_state = false;
  delay(1000);
  }
  }
}





