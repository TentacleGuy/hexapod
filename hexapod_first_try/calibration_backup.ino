
//***********************************************************************
// Angle calibration of servos
//***********************************************************************
void calibration(){
  Serial.println("kakalibriert");
  int calibrating = 1;
  int leg = 0;
  int servo = 0;
  while(calibrating == 1){
    if (ps2x.ButtonPressed(PSB_PAD_UP) && leg < 4)    //select leg
    {
      leg++;
      Serial.print("Bein: ");
      Serial.println(leg);
    }
    else if (ps2x.ButtonPressed(PSB_PAD_DOWN) && leg > 1)    //select leg
    {
      leg--;
      Serial.print("Bein: ");
      Serial.println(leg);
    }
    else if (ps2x.ButtonPressed(PSB_PAD_RIGHT) && servo > 1)    //select leg
    {
      servo--;
      Serial.print("Gelenk: ");
      Serial.println(servo);
    }
    else if (ps2x.ButtonPressed(PSB_PAD_LEFT) && servo > 2)    //select leg
    {
      servo++;
      Serial.print("Gelenk: ");
      Serial.println(servo);
    }
    else if (ps2x.ButtonPressed(PSB_L1)){
      switch(servo){
        case 0:
          COXA_CAL[leg]++;                      
        break;
        case 1:
          FEMUR_CAL[leg]++; 
        break;
        case 2:
          TIBIA_CAL[leg]++;
        break;
        default:
          Serial.println("errorServo");
        break;
      }  
    }
    else if (ps2x.ButtonPressed(PSB_L2)){
        switch(servo){
        case 0:
          COXA_CAL[leg]--;                       //servo calibration variables
        break;
        case 1:
          FEMUR_CAL[leg]--; 
        break;
        case 2:
          TIBIA_CAL[leg]--;
        break;
        default:
          Serial.println("errorServo");
        break;
      }
    }
    else if (ps2x.ButtonPressed(PSB_R1)){
      calibrating = 0;
      break;
    }
    //
  }
}