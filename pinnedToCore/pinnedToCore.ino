#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include <Arduino_FreeRTOS.h>
//#include <semphr.h>

//using only core 1 for demo purpose
#if CONFIG_FREETOS_UNICORE
static const BaseType_t app_cpu =0;
#else
static const BaseType_t app_cpu=1;
#endif

// Globals
static QueueHandle_t delay_queue;
static QueueHandle_t msg_queue;


//LED_BUILTIN
static const int led_pin =19;

//const char msg[]="Barkadeerbri arr booty rum";

static TaskHandler_t task_1 = NULL;
static TaskHandler_t task_2 = NULL;

SemaphoreHandle_t  sema_v; 
sema_v = xSemaphoreCreateBinary();

void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600);


    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println();
    Serial.println("freertos task demo");
            
    Serial.println("setup and loop task running on core");
    Serial.println(xPortGetCoreID());
    Serial.println("with priority ");
    Serial.println(uxTaskPriorityGet(NULL));
    
  

    
    xTaskCreatePinnedToCore(
      startTask1,//use xtaskcreat in vanilla freetos
    "one", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    1, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_1,
    app_cpu); //run on one core for demo 

        //task to run forever
    xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "two", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask3,//use xtaskcreat in vanilla freetos
    "three", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask5,//use xtaskcreat in vanilla freetos
    "four", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "five", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "six", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "seven", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "eight", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

        xTaskCreatePinnedToCore(
      startTask2,//use xtaskcreat in vanilla freetos
    "nine", //function to be called
    1024, //stack size (bytes in esp32, words in freetos) 1kB
    NULL, //paramters to pass to function
    2, //task priority 0 to 24, 0 lowest - can set pointer or handle so that can see what the status of this task is, check memor usae
    &task_2, //assigns the handle
    app_cpu); //run on one core for demo 

    
    

      // Create queues
  delay_queue = xQueueCreate(delay_queue_len, sizeof(int));
  msg_queue = xQueueCreate(msg_queue_len, sizeof(Message));


    interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    attachInterrupt(digitalPinToInterrupt(2), debounceInterrupt, LOW);
  }
  
}

void one() {
  //this is the watchdog waveform task
  digitalWrite(SignalB, HIGH);
  delayMicroseconds(50);
  digitalWrite(SignalB, LOW);
  delayMicroseconds(50);
}

int digitalSwitchState = 0;
void two() {
  //readinging digital switch state into the global array
  digitalSwitchState = digitalRead(pushButton1);

}

int oscilloscopeFrequency = 0;
void three() {
  
  float mark1;
  mark1 = pulseIn(squareWave, HIGH); //output how long the input signal is high
  float period = mark1 * 2; //is in microseconds
  float freq = 1e6 / period; //dividing by 1e6 as this is 1micro in second as you need to scale it
  oscilloscopeFrequency = freq;

}

//creating global variables which are accessed for potentiometer average value
int firstFour = 1;
float arrayValue[] = {0, 0, 0, 0};
void four() {

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

}

int global_average = 0;
void five() {
  int local_average = 0;
  //for looping over certain amount of values, allows average of 1,2,3 and not average initial 0
  for (int i = 0; i < firstFour; i++) {
    //sum values
    local_average = local_average + arrayValue[i];
  }
  //divide by the amount of values in the array
  local_average = local_average / firstFour;
  global_average = local_average;
}

void six() {
  //no op task, does it required amount of times
  for (int i = 0; i < 1000; i++) {
    __asm__ __volatile__ ("nop");
  }
}

int error_code = 0;
void seven() {
  //if the average of the analog pot is larger than the half of the max value put LED on otherwise put it off
  if (global_average > (potMax / 2)) {
    error_code = 1;
  } else {
    error_code = 0;
  }

}

void eight() {
  //condition for led code taking global variable and if there is an error then put led on
  if (error_code)
    digitalWrite(LEDerrorCode, HIGH);
  else
    digitalWrite(LEDerrorCode, LOW);
}

void nine() {
  //outputting to serial for csv
  Serial.print(digitalSwitchState);
  Serial.print(",");

  Serial.print(oscilloscopeFrequency);
  Serial.print(",");

  Serial.print(global_average);
  Serial.println(",");


}



//void startTask1(void *parameter){
//  //returns void pointer parameter
//  int msg_len =strlen(msg);
//  while(1){
//    Serial.println();
//    for(int i=0; i< msg_len;i++){
//       Serial.println(msg[i]);
//    }
//     Serial.println();
//     vTaskDelay(1000 / portTick_PERIOD_MS);
//  }
//}
//
//void startTask2(void *parameter){
//
//  while(1){
//    Serial.println("*");
//     vTaskDelay(100 / portTick_PERIOD_MS);
//  }
//}




void loop() {
  // put your setup code here, to run once:
  for (int i=0; i<3; i++){ //suspend task 2 which is sending out the asterisk every 2
    vTaskSuspend(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  if (task_1 != NULL){
    vTaskDelete(task_1);//need to make sure to check whether the task exists first before you delete it, might overwrite memory
    task_1=NULL;
  }
}
