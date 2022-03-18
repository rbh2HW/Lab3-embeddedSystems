#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include <Arduino_FreeRTOS.h>
//#include <semphr.h>

//using only core 1 for demo purpose
#if CONFIG_FREETOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Globals
static QueueHandle_t delay_queue;
static QueueHandle_t msg_queue;


//LED_BUILTIN
static const int led_pin = 19;

//const char msg[]="Barkadeerbri arr booty rum";

//static TaskHandler_t task_1 = NULL;
//static TaskHandler_t task_2 = NULL;

//SemaphoreHandle_t  sema_v;
//sema_v = xSemaphoreCreateBinary();
#define potMax 4095 //setting max potentiometer value

#define SignalB 21 //GPIO21
#define pushButton1 23 //GPIO23
#define analogPot 35 //GPIO35
#define LEDerrorCode 25 //gpio 25

#define squareWave 34 //GPIO22


//typedef struct
//{
//  unsigned int 
//  
//}globaltask9Struct


int period1 = 16;
int period2 = 200;
int period3 = 1000;
int period4 = 42;
int period5 = 42;
int period6 = 100;
int period7 = 333;
int period8 = 333;
int period9 = 5000;


void one(void *parameter){
  while (true) {
    //this is the watchdog waveform task
    digitalWrite(SignalB, HIGH);
    delayMicroseconds(50);
    digitalWrite(SignalB, LOW);
    delayMicroseconds(50);
    vTaskDelay(period1 / portTICK_PERIOD_MS);
  }
}

int digitalSwitchState = 0;
void two(void *parameter){
  while (true) {
    Serial.println("task1");
    //readinging digital switch state into the global array
    digitalSwitchState = digitalRead(pushButton1);
    vTaskDelay(period2 / portTICK_PERIOD_MS);
  }
}

int oscilloscopeFrequency = 0;
void three(void *parameter){
  while (true) {
    float mark1;
    mark1 = pulseIn(squareWave, HIGH); //output how long the input signal is high
    float period = mark1 * 2; //is in microseconds
    float freq = 1e6 / period; //dividing by 1e6 as this is 1micro in second as you need to scale it
    oscilloscopeFrequency = freq;
    vTaskDelay(period3 / portTICK_PERIOD_MS);
  }
}

//creating global variables which are accessed for potentiometer average value
int firstFour = 1;
float arrayValue[] = {0, 0, 0, 0};
void four(void *parameter){
  while (true) {

    //initiate and read the pot value
    float potValue;
    potValue = analogRead(analogPot);


    if (firstFour < 4) {
      //this adds index and number for the first 4 values obtained for the average
      //this is an edge condition as there are 4 0 values in the global array
      arrayValue[(firstFour - 1)] = potValue;
      firstFour++;

    } else {
      //if there are 4 values in the array do this
      //with array delete the last value from it and add the new value at index 1
      arrayValue[3] = arrayValue[2];
      arrayValue[2] = arrayValue[1];
      arrayValue[1] = arrayValue[0];
      arrayValue[0] = potValue;

    }
    vTaskDelay(period4 / portTICK_PERIOD_MS);
  }
}

int global_average = 0;
void five(void *parameter){
  while (true) {
    int local_average = 0;
    //for looping over certain amount of values, allows average of 1,2,3 and not average initial 0
    for (int i = 0; i < firstFour; i++) {
      //sum values
      local_average = local_average + arrayValue[i];
    }
    //divide by the amount of values in the array
    local_average = local_average / firstFour;
    global_average = local_average;
    vTaskDelay(period5 / portTICK_PERIOD_MS);
  }
}

void six(void *parameter){
  while (true) {
    //no op task, does it required amount of times
    for (int i = 0; i < 1000; i++) {
      __asm__ __volatile__ ("nop");
    }
    vTaskDelay(period6 / portTICK_PERIOD_MS);
  }
}

int error_code = 0;
void seven(void *parameter){
  while (true) {
    //if the average of the analog pot is larger than the half of the max value put LED on otherwise put it off
    if (global_average > (potMax / 2)) {
      error_code = 1;
    } else {
      error_code = 0;
    }
    vTaskDelay(period7 / portTICK_PERIOD_MS);
  }
}

void eight(void *parameter){
  while (true) {
    //condition for led code taking global variable and if there is an error then put led on
    if (error_code)
      digitalWrite(LEDerrorCode, HIGH);
    else
      digitalWrite(LEDerrorCode, LOW);
    vTaskDelay(period8 / portTICK_PERIOD_MS);
  }
}

void nine(void *parameter){
  while (true) {
    //outputting to serial for csv
    Serial.print(digitalSwitchState);
    Serial.print(",");

    Serial.print(oscilloscopeFrequency);
    Serial.print(",");

    Serial.print(global_average);
    Serial.println(",");
    vTaskDelay(period9 / portTICK_PERIOD_MS);
  }
}




void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(SignalB, OUTPUT); //signalB from 21
  pinMode(LEDerrorCode, OUTPUT);

  pinMode(pushButton1, INPUT); //disable stream pulse button
  pinMode(analogPot, INPUT); //mode selection button
  pinMode(squareWave, INPUT);


  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("freertos task demo");

  Serial.println("setup and loop task running on core");
  Serial.println(xPortGetCoreID());
  Serial.println("with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  //    xTaskCreatePinnedToCore(

  xTaskCreate(one,  "taskone", 1024, NULL, 0, NULL );
  xTaskCreate(two,  "tasktwo", 1024, NULL, 0, NULL );
  xTaskCreate(three,  "taskthree", 1024, NULL, 0, NULL );
  xTaskCreate(four,  "taskfour", 1024, NULL, 0, NULL );
  xTaskCreate(five,  "taskfive", 1024, NULL, 0, NULL );
  xTaskCreate(six,  "tasksix", 1024, NULL, 0, NULL );
  xTaskCreate(seven,  "taskseven", 1024, NULL, 0, NULL );
  xTaskCreate(eight,  "taskeight", 1024, NULL, 0, NULL );
  xTaskCreate(nine,  "tasknine", 1024, NULL, 0, NULL );

  // Create queues
//  delay_queue = xQueueCreate(delay_queue_len, sizeof(int));
//  msg_queue = xQueueCreate(msg_queue_len, sizeof(Message));

}


void loop() {



  // put your setup code here, to run once:
  //  for (int i=0; i<3; i++){ //suspend task 2 which is sending out the asterisk every 2
  //    vTaskSuspend(task_2);
  //    vTaskDelay(2000 / portTICK_PERIOD_MS);
  //    vTaskResume(task_2);
  //    vTaskDelay(2000 / portTICK_PERIOD_MS);
  //  }
  //  if (task_1 != NULL){
  //    vTaskDelete(task_1);//need to make sure to check whether the task exists first before you delete it, might overwrite memory
  //    task_1=NULL;
  //  }
}
