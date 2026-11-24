#include "botSetup/vex.h"
#include "Libraries/controllerUI.h"
#include <string>

int reprint(controller CONTROLLER, int index, std::string b1, std::string b2, std::string b3){
  if(b1!="NODISPLAY"){
    CONTROLLER.Screen.clearLine(1);
  }
  if(b2!="NODISPLAY"){
    CONTROLLER.Screen.clearLine(2);
  } 
  if(b2!="NODISPLAY"){
    CONTROLLER.Screen.clearLine(3);
  }

  if(b1!=""&&b1!="NODISPLAY"){ 
    CONTROLLER.Screen.setCursor(1, 3);
    CONTROLLER.Screen.print(b1.c_str());
  }
  if(b2!=""&&b2!="NODISPLAY"){
    CONTROLLER.Screen.setCursor(2, 3);
    CONTROLLER.Screen.print(b2.c_str());
  }
  if(b3!=""&&b3!="NODISPLAY"){
    CONTROLLER.Screen.setCursor(3, 3);
    CONTROLLER.Screen.print(b3.c_str());
  }

  CONTROLLER.Screen.setCursor(index, 1);
  CONTROLLER.Screen.print("->");

  return index;
}

std::string controllerSelector(controller CONTROLLER, std::string b1, std::string b2, std::string b3){
  int maxIndex = 0;
  int noDisplay = 0;
  if(b1!=""&&b1!="NODISPLAY"){
    maxIndex++;
  }
  else if(b1=="NODISPLAY"){
    noDisplay++;
  }

  if(b2!=""&&b2!="NODISPLAY"){
    maxIndex++;
  }
  else if(b2=="NODISPLAY"){
    noDisplay++;
  }

  if(b3!=""&&b3!="NODISPLAY"){
    maxIndex++;
  }
  else if(b3=="NODISPLAY"){
    noDisplay++;
  }
  
  int selected = 1;
  int showSelected = 1;

  reprint(CONTROLLER, selected+noDisplay, b1, b2, b3);
  while(!CONTROLLER.ButtonA.pressing()){
    if(selected!=showSelected){
      reprint(CONTROLLER, selected+noDisplay, b1, b2, b3);
      showSelected=selected;
    }
    if(CONTROLLER.ButtonUp.pressing()){
      selected--;
      if(selected<1){
        selected=maxIndex;
      }
    }
    if(CONTROLLER.ButtonDown.pressing()){
      selected++;
      if(selected>maxIndex){
        selected=1;
      }
    }

    if(CONTROLLER.ButtonB.pressing()){
      return "EXIT";
    }
    
    wait(10, msec);
  }
  
  if(selected+noDisplay==1){
    return b1;
  }
  if(selected+noDisplay==2){
    return b2;
  }
  if(selected+noDisplay==3){
    return b3;
  }

  return "ERROR";

}



std::string controllerSelectorEXT(controller CONTROLLER, int length, std::string b[length]){
  int maxScroll = length-3;
  int noDisplay = 0;
  // if(b1!=""&&b1!="NODISPLAY"){
  //   maxIndex++;
  // }
  // else if(b1=="NODISPLAY"){
  //   noDisplay++;
  // }

  // if(b2!=""&&b2!="NODISPLAY"){
  //   maxIndex++;
  // }
  // else if(b2=="NODISPLAY"){
  //   noDisplay++;
  // }

  // if(b3!=""&&b3!="NODISPLAY"){
  //   maxIndex++;
  // }
  // else if(b3=="NODISPLAY"){
  //   noDisplay++;
  // }
  
  int scroll=0;
  int selected = 1;
  int showSelected = 1;

  reprint(CONTROLLER, selected+noDisplay, b[scroll], b[scroll+1], b[scroll+2]);

  while(!CONTROLLER.ButtonA.pressing()){
    if(CONTROLLER.ButtonUp.pressing()){
      selected--;
      if(selected<1){
        scroll--;
        if(scroll<0){
          scroll=maxScroll;
          selected=3;
        }
        else{
          selected=1;
        }
      }

      reprint(CONTROLLER, selected+noDisplay, b[scroll], b[scroll+1], b[scroll+2]);
    }
    if(CONTROLLER.ButtonDown.pressing()){
      selected++;

      if(selected>3){
        scroll++;
        if(scroll>maxScroll){
          scroll=0;
          selected=1;
        }
        else{
          selected=3;
        }
      }

      reprint(CONTROLLER, selected+noDisplay, b[scroll], b[scroll+1], b[scroll+2]);
    }

    wait(10,msec);

  }

  return b[selected-1+scroll];

  //return "ERROR";

}

std::string confirmControllerSelector(controller CONTROLLER, std::string b1, std::string b2, std::string b3, bool title, std::string confText){
  bool confirmed=false;
  while(confirmed==false){
    std::string selected;
    if(title==true){
      CONTROLLER.Screen.clearScreen();
      CONTROLLER.Screen.setCursor(1, 1);
      CONTROLLER.Screen.print(b3.c_str());
      selected = controllerSelector(CONTROLLER, "NODISPLAY", b1, b2);
    }
    else{
      selected = controllerSelector(CONTROLLER, b1, b2, b3);
    }
    CONTROLLER.Screen.clearScreen();
    CONTROLLER.Screen.setCursor(1, 1);
    CONTROLLER.Screen.print(confText.c_str());

    std::string confirmString = controllerSelector(CONTROLLER, "NODISPLAY", selected, "CANCEL");

    if(confirmString==selected){
      confirmed=true;
      return selected;
    }
    if(confirmString=="CANCEL"){
      confirmed=false;
    }
  }
  return "ERROR";
}

void displayInfoSync(controller CONTROLLER, std::string l1, std::string l2, std::string l3){
 CONTROLLER.Screen.clearScreen();
 CONTROLLER.Screen.setCursor(1, 1);
 CONTROLLER.Screen.print(l1.c_str());
 CONTROLLER.Screen.setCursor(2, 1);
 CONTROLLER.Screen.print(l2.c_str());
 CONTROLLER.Screen.setCursor(3, 1);
 CONTROLLER.Screen.print(l3.c_str());
}


// Structure to hold display information
struct DisplayData {
  controller* CONTROLLER;
  std::string l1;
  std::string l2;
  std::string l3;
};

bool displaying = false;

//Display function that runs async
int displayInfoTask(void* arg) {
  //Ensure it doesn't run twice at once
  if(displaying == true){
    return 0;
  }
  displaying = true;

  //Unpack the memory referenced
  DisplayData* data = static_cast<DisplayData*>(arg);
  
  //Clear screen and display like one
  data->CONTROLLER->Screen.clearScreen();
  data->CONTROLLER->Screen.setCursor(1, 1);
  data->CONTROLLER->Screen.print(data->l1.c_str());
  if(data->l2 == "" && data->l3 == ""){//Exit if there is no line 2 or 3
    delete data; // Clean up the allocated memory
    displaying = false; 
    return 0;
  }
  data->CONTROLLER->Screen.setCursor(2, 1);//Display line 2
  data->CONTROLLER->Screen.print(data->l2.c_str());
  if(data->l3 == ""){ //Exit if there is no line 3
    delete data; // Clean up the allocated memory
    displaying = false; 
    return 0;
  }
  data->CONTROLLER->Screen.setCursor(3, 1);//Display line 3
  data->CONTROLLER->Screen.print(data->l3.c_str());

  //exit
  delete data; // Clean up the allocated memory
  displaying = false; 
  return 0;
}

// The wrapper function that creates and starts the task
void displayInfo(controller CONTROLLER, std::string l1, std::string l2, std::string l3) {
  //Load the display strings to struct
  DisplayData* data = new DisplayData{&CONTROLLER, l1, l2, l3};

  //Supply the task with the pointer to the data
  vex::task null = task(displayInfoTask, data);
}