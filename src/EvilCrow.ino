#include <Arduino.h>
#include <FreeRTOS_Shell.h>

void toggleLED(void * parameter){
  int led = 32;
  pinMode(led, OUTPUT);
  for(;;){ // infinite loop

    // Turn the LED on
    digitalWrite(led, HIGH);

    // Pause the task for 500ms
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    // Turn the LED off
    digitalWrite(led, LOW);

    // Pause the task again for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  xTaskCreate(
    toggleLED,    // Function that should be called
    "Toggle LED",   // Name of the task (for debugging)
    1008,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );

  xTaskCreate(
    FreeRTOS_Shell,
    "Shell",
    4096,
    NULL,
    10,
    NULL
  );


}

void loop() {
  delay(10);
}

