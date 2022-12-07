void LineFollower(int distanceTH){

  int distance = obstacle(trigPin1, echoPin1);

  if (distance <= distanceTH) {

    // Stop Zekoo
    angleV = 0;
    turnV = 0;

    // Arm control functions
  // ArmPos(&pos1, &pos2, &pos3, &angle1, &angle2, &angle3);

  }else{

    // Line follower 
    bool value1 = digitalRead(IR1_Pin);
    bool value2 = digitalRead(IR2_Pin);

    if (value1 == 0 && value2 == 0) {
      angleV = 10;
    } else if (value1 == 1 && value2 == 1) {
      angleV = 0;
      turnV  = 0;
    } else if (value1 == 1 && value2 == 0) {
      turnV = 25;
    } else if (value1 == 0 && value2 == 1) {
     turnV = -25;
    }

  }

}
