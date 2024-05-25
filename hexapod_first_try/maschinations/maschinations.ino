#include <Adafruit_NeoPixel.h>

const int LED_PINS[] = { 4,5 };
const int LEDS_COUNT = 7;
Adafruit_NeoPixel LEDS_LEG[2];
const int MAX_LEGS = sizeof(LEDS_LEG) / sizeof(LEDS_LEG[0]);
const int FRAME_TIME_MS = 20;
const int IDLE_TIME_MS = 4000;

int animParams[10];
int animationStep = 0;
bool booted = false;

unsigned long previousTime;
unsigned long previousAnimTime;
unsigned long lastAction;

int demo = 0;
int buttonPressed = 0;
static unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 200;
int lastBtnState = LOW;
bool isAnimationActive = false;

enum AnimationState {
  ANIM_INIT,
  ANIM_RUNNING,
  ANIM_DONE
};

AnimationState bootupState = ANIM_INIT;
AnimationState carouselState = ANIM_INIT;
AnimationState idleState = ANIM_INIT;

void setup() {
  Serial.begin(115200);
  pinMode(8, INPUT_PULLUP);

  for (int i = 0; i < MAX_LEGS; i++) {
    LEDS_LEG[i] = Adafruit_NeoPixel(LEDS_COUNT, LED_PINS[i], NEO_GRB + NEO_KHZ800);
    LEDS_LEG[i].setBrightness(32);
    LEDS_LEG[i].begin();
    LEDS_LEG[i].show();
  }

  lastAction = millis();
}

void bootupAnimation(int params[], int &step, AnimationState &state) {
  static unsigned long lastUpdateTime = 0;
  if (state == ANIM_INIT) {
    step = 0;
    lastUpdateTime = 0;
    state = ANIM_RUNNING;
    Serial.println("Bootup Animation: INIT");
  }

  Serial.print("Step: ");
  Serial.println(step);
  Serial.print("State: ");
  Serial.println(state);
  Serial.print("millis(): ");
  Serial.println(millis());
  Serial.print("lastUpdatedTime: ");
  Serial.println(lastUpdateTime);
  Serial.print("params4: ");
  Serial.println(params[4]);

  if (state == ANIM_RUNNING && millis() - lastUpdateTime > params[4]) {
    lastUpdateTime = millis();
    for (int n = 0; n < MAX_LEGS; n++) {
      if (step < LEDS_COUNT) {
        LEDS_LEG[n].setPixelColor(step, LEDS_LEG[n].Color(params[1], params[2], params[3]));
      } else {
        LEDS_LEG[n].setPixelColor(step - LEDS_COUNT, LEDS_LEG[n].Color(0, 0, 0));
      }
      LEDS_LEG[n].show();
    }
    step++;

    if (step >= LEDS_COUNT * 2) {
      state = ANIM_DONE;
      isAnimationActive = false;
      Serial.println("Bootup Animation: DONE");
    }
  }
}


void offAnimation() {
  for (int n = 0; n < MAX_LEGS; n++) {
    LEDS_LEG[n].clear();
    LEDS_LEG[n].show();
  }
}

void carouselAnimation(int params[], int &step, AnimationState &state) {
  static unsigned long lastUpdateTime = 0;
  if (state == ANIM_INIT) {
    step = 0;
    state = ANIM_RUNNING;
  }

  if (state == ANIM_RUNNING && millis() - lastUpdateTime > params[4]) {
    lastUpdateTime = millis();
    for (int n = 0; n < MAX_LEGS; n++) {
      for (int i = 0; i < LEDS_COUNT; i++) {
        if (n == step) {
          LEDS_LEG[n].setPixelColor(i, LEDS_LEG[n].Color(params[1], params[2], params[3]));
        } else {
          LEDS_LEG[n].setPixelColor(i, LEDS_LEG[n].Color(0, 0, 0));
        }
      }
      LEDS_LEG[n].show();
    }

    step++;
    if (step >= MAX_LEGS) {
      state = ANIM_DONE;
      isAnimationActive = false;
    }
  }
}

void idleAnimation(int params[], int &step, AnimationState &state) {
  static unsigned long lastUpdateTime = 0;
  if (state == ANIM_INIT) {
    step = 0;
    state = ANIM_RUNNING;
  }

  if (state == ANIM_RUNNING && millis() - lastUpdateTime > 5) {
    lastUpdateTime = millis();
    for (int n = 0; n < MAX_LEGS; n++) {
      for (int i = 0; i < LEDS_COUNT; i++) {
        int colorIndex = ((LEDS_COUNT - 1 - i) * 256 / LEDS_COUNT + step) & 255;
        LEDS_LEG[n].setPixelColor(i, Wheel(colorIndex, n));
      }
      LEDS_LEG[n].show();
    }
    step = (step + 1) % 256;
  }
  /*
  if (button(1) == 1) {
    lastAction = millis();
    state = ANIM_DONE;
    isAnimationActive = false;
  }*/
}

uint32_t Wheel(byte WheelPos, int n) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return LEDS_LEG[n].Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return LEDS_LEG[n].Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return LEDS_LEG[n].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void setArray(int params[], int numArgs, ...) {
  va_list args;
  va_start(args, numArgs);

  for (int i = 0; i < numArgs; i++) {
    params[i] = va_arg(args, int);
  }

  va_end(args);
}
void playdemo(int i) {
  if (isAnimationActive) {
    Serial.println("Animation läuft bereits.playdemo");
    return;
  }

  //offAnimation();
  switch (i) {
    case 1:
      Serial.println("demo1");
      setArray(animParams, 5, 0, 255, 0, 0, 50);
      bootupState = ANIM_INIT;
      bootupAnimation(animParams, animationStep, bootupState);
      isAnimationActive = true;
      break;
    case 2:
    Serial.println("demo2");
      setArray(animParams, 5, 1, 0, 255, 0, 100);
      carouselState = ANIM_INIT;
      carouselAnimation(animParams, animationStep, carouselState);
      isAnimationActive = true;
      break;
    default:
      demo = 0;
      break;
  }
}

int button(int PR) {  //Tasterdruckabfrage mit Entprellung PR = 0 für Release und 1 für Druck
  static unsigned long lastButtonPressTime = 0;
  const unsigned long debounceDelay = 200;

  if (digitalRead(8) == HIGH && lastBtnState == LOW) {
    if (millis() - lastButtonPressTime >= debounceDelay) {
      lastBtnState = HIGH;
      if (PR == 1) {
        lastButtonPressTime = millis();
        return 1;
      }
    }
  } else if (digitalRead(8) == LOW && lastBtnState == HIGH) {
    if (millis() - lastButtonPressTime >= debounceDelay) {
      lastBtnState = LOW;
      if (PR == 0) {
        lastButtonPressTime = millis();
        
        return 1;
      }
    }
  }

  return -1;
}

void loop() {

  if (demo >= 2) {
    demo = 0;
  } 
  
  if (button(1) != -1 && !isAnimationActive) {
    Serial.println("buttonpressed");
    lastAction = millis();
    demo++;
    playdemo(demo);
  }
/*
  if ((millis() - previousTime) > FRAME_TIME_MS) {
    previousTime = millis();
    if (!booted && !isAnimationActive) {
      Serial.println("bootstart");
      setArray(animParams, 5, 0, 255, 0, 0, 20);
      bootupAnimation(animParams, animationStep, bootupState);
      isAnimationActive = true;
    }

    if ((millis() - lastAction) > IDLE_TIME_MS && !isAnimationActive) {
      Serial.println("idlestart");
      idleAnimation(animParams, animationStep, idleState);
      isAnimationActive = true;
    }
  }*/
}
