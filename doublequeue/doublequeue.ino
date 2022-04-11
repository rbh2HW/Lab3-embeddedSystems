#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//average queue sent to task 7 LED
static const uint8_t GlobalAverageQueueLED_queue_len = 8;
static QueueHandle_t GlobalAverageQueueLED;

//used to store the pot values for task 5
static const uint8_t averageElements_queue_len = 5;
static QueueHandle_t averageElements;



//LED_BUILTIN
static const int led_pin = 19;


//setting up mutex variable
static SemaphoreHandle_t mut_sem;


#define potMax 4095 //setting max potentiometer value

#define SignalB 21 //GPIO21
#define pushButton1 23 //GPIO23
#define analogPot 35 //GPIO35
#define LEDerrorCode 25 //gpio 25
#define squareWave 34 //GPIO34


//creating the critical storage struct
struct taskStorage
{
  unsigned int global_average = 0;
  unsigned int digitalSwitchState = 0;
  unsigned int oscilloscopeFrequency = 0;
};

taskStorage globaltask9Struct; //creating globaltask9struct as the variable going to be used

//creating the periods for each task, how often they should be executed
static int period1 = 16;
static int period2 = 200;
static int period3 = 1000;
static int period4 = 42;
static int period5 = 42;
static int period6 = 100;
static int period7 = 333;
static int period8 = 333;
static int period9 = 5000;



void one(void *parameter) {
  while (true) {

    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back
    //this is the watchdog waveform task
    digitalWrite(SignalB, HIGH);
    delayMicroseconds(50);
    digitalWrite(SignalB, LOW);
    delayMicroseconds(50);
    vTaskDelay(period1 / portTICK_PERIOD_MS);
  }
}

//
void two(void *parameter) {
  int localdigitalSwitchState = 0;
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    //readinging digital switch state into the global array
    localdigitalSwitchState = digitalRead(pushButton1);

    //takes and gives the task9 semaphore and adds the current value of the digital switch
    xSemaphoreTake(mut_sem, portMAX_DELAY);
    globaltask9Struct.digitalSwitchState = localdigitalSwitchState;
    xSemaphoreGive(mut_sem);

    vTaskDelay(period2 / portTICK_PERIOD_MS);
  }
}

//
void three(void *parameter) {
  
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back
    float mark1;
    mark1 = pulseIn(squareWave, HIGH); //output how long the input signal is high
    float period = mark1 * 2; //is in microseconds
    float freq = 1e6 / period; //dividing by 1e6 as this is 1micro in second as you need to scale it
    

    //takes and gives the task9 semaphore and adds the current value of the oscilloscope frequency
    xSemaphoreTake(mut_sem, portMAX_DELAY);
    globaltask9Struct.oscilloscopeFrequency = freq;
    xSemaphoreGive(mut_sem);

    vTaskDelay(period3 / portTICK_PERIOD_MS);
  }
}

//creating global variables which are accessed for potentiometer average value
int firstFour = 1;

void four(void *parameter) {

  int potValue;//analog pot value
  int throwaway; //need to intantiate variable such that can through variable away if there are 4 variables in queue

  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    //initiate and read the pot value

    potValue = analogRead(analogPot);

    //i want to average 4 values but don't want to take out a value if there aren't 4 values, therefore if firstFour is less than 4 then don't remove an element, otherwise remove an element and add one
    if (firstFour < 4) {
      //this adds index and number for the first 4 values obtained for the average
      //this is an edge condition as there are 4 0 values in the global array
      
      firstFour++;
      if (xQueueSend(averageElements, (void *)&potValue, 2) != pdTRUE) {
        Serial.println("ERROR: Could not put item on averageElementsFirstFour queue.");
      } else {
        //        Serial.println("ERROR: Could not put item on delay queue.");
      }

    } else {
      //if there are 4 values in the array do this
      //with array delete the last value from it and add the new value at index 1

      if (xQueueReceive(averageElements, (void *)&throwaway, 0) != pdTRUE) {//taking element away from queue
        Serial.println("ERROR: Could not pop from average queue");
      }
      if (xQueueSend(averageElements, (void *)&potValue, 2) != pdTRUE) {//adding new element for queue
        Serial.println("ERROR: Could not put item on average queue.");
      }

    }
    vTaskDelay(period4 / portTICK_PERIOD_MS);
  }
}


void five(void *parameter) {

  int arrayValue;
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    int tempValueArray;
    int tempArray[] = {0, 0, 0, 0};
    int global_average = 0;

    int local_average = 0;

    //for looping over certain amount of values, allows average of 1,2,3 and not average initial 0
    for (int i = 0; i < firstFour; i++) {

      //iteratively taking value from queue from arrayElements, setting returned value to arrayValue then adding to local_average
      if (xQueueReceive(averageElements, (void *)&arrayValue, 0) != pdTRUE) {
        Serial.println("ERROR: Could not pop from average queue");
      }

      local_average = local_average + arrayValue; //caculating average of queue

      tempArray[i] = arrayValue; //adding elements from queue to array so that it can be added back later

    }



    //divide by the amount of values in the array
    local_average = local_average / firstFour;
    global_average = local_average;


    //sending to queue, queue name, local variable name, ticks on how long to wait
    if (xQueueSend(GlobalAverageQueueLED, (void *)&global_average, 2) != pdTRUE) {
      Serial.println("ERROR: Could not put item on delay queue.");//send message if not able to send to queue
    }
    //    struct globaltask9Struct localglobaltask9Struct;

    //updating the task9 value by taking and giving the semaphore
    xSemaphoreTake(mut_sem, portMAX_DELAY);
    globaltask9Struct.global_average = global_average; //store global average
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    
    //this is putting the elements back in the queue with the new value
    for (int i = 0; i < firstFour; i++) {

      //setting tempValueArray to a current value that you want to send back to the queue
      tempValueArray = tempArray[i];
      
      if (xQueueSend(averageElements, (void *)&tempValueArray, 2) != pdTRUE) {
        Serial.println("ERROR: Could not put item back on average queue.");
      }

    }

    vTaskDelay(period5 / portTICK_PERIOD_MS);
  }
}

void six(void *parameter) {
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    //no op task, does it required amount of times
    for (int i = 0; i < 1000; i++) {
      __asm__ __volatile__ ("nop");
    }
    vTaskDelay(period6 / portTICK_PERIOD_MS);
  }
}

int error_code = 0;
void seven(void *parameter) {
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    int global_average;
    for (int i = 0; i < 8; i++) {//go over 8 values of queue from 24hz to 3hz difference

      if (xQueueReceive(GlobalAverageQueueLED, (void *)&global_average, 0) == pdTRUE) {//taking the 8 values from the queue
        //        Serial.println(i);
        if (global_average > (potMax / 2)) {
          error_code = 1;
        } else {
          error_code = 0;
        }
      }

      //if the average of the analog pot is larger than the half of the max value put LED on otherwise put it off

    }

    vTaskDelay(period7 / portTICK_PERIOD_MS);
  }
}

void eight(void *parameter) {
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    //condition for led code taking global variable and if there is an error then put led on
    if (error_code)
      digitalWrite(LEDerrorCode, HIGH);
    else
      digitalWrite(LEDerrorCode, LOW);
    vTaskDelay(period8 / portTICK_PERIOD_MS);
  }
}

void nine(void *parameter) {
  while (true) {
    xSemaphoreGive(mut_sem); //errors were given if it wasn't sure that the semaphore wasn't given back

    struct taskStorage localglobaltask9Struct;

    xSemaphoreTake(mut_sem, portMAX_DELAY);
    localglobaltask9Struct = globaltask9Struct;
    xSemaphoreGive(mut_sem);

    if (localglobaltask9Struct.digitalSwitchState) {//only print task9 if button is pressed


      //outputting to serial for csv
      Serial.print(localglobaltask9Struct.digitalSwitchState);
      Serial.print(",");

      Serial.print(localglobaltask9Struct.oscilloscopeFrequency);
      Serial.print(",");

      Serial.print(localglobaltask9Struct.global_average);
      Serial.println(",");

    }

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


  //  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("freertos task demo");

  Serial.println("setup and loop task running on core");
  Serial.println(xPortGetCoreID());
  Serial.println("with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  //create both queues, globalaveragequeue is for
  GlobalAverageQueueLED = xQueueCreate(GlobalAverageQueueLED_queue_len, sizeof(int));
  averageElements = xQueueCreate(averageElements_queue_len, sizeof(int));


  //setup and create mux
  mut_sem = xSemaphoreCreateMutex();


  //create tasks not assigned to any cores
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

}


void loop() {



  
}
